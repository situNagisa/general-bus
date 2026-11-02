/*
	* multi_sqe.cpp — 一次 io_uring_submit 提交多个 SQE，每个操作各自完成
	*
	* 一、这是什么、解决什么问题
	* 上一节 single_sqe.cpp 只提交了一个操作。io_uring 真正的价值在于“批量”：
	* 应用可以在 SQ 环里连续填好 N 个 SQE，只调用一次 io_uring_submit()，
	* 内核就接管这 N 个请求并并行推进。N 个请求互不依赖，谁先完成谁先产生
	* CQE，完成顺序和提交顺序没有对应关系。
	*
	* 二、形态（本例）
	*   - 一次 io_uring_submit() 提交 4 个 SQE：2 个 recv + 2 个 send；
	*   - 每个 SQE 对应 1 次内核操作，各产生 1 个 CQE，共 4 个；
	*   - CQE 到达顺序不确定：可能 send 先完成，也可能 recv 先完成；
	*   - 靠 SQE 里的 user_data 把 CQE 认领回具体请求，绝不能靠“第几个
	*     CQE 就是第几个请求”；
	*   - 每个请求有自己的缓冲区（两个 recv 各一块），由应用提供并保活；
	*   - 本例用 SOCK_DGRAM 的 socketpair，保证一次 recv 正好取到一条消息
	*     （流式套接字会粘包/短读，见文末说明）。
	*
	* 三、内核条件
	*   - 内核 >= 5.1；不需要特殊硬件。
	*
	* 四、编译与运行
	*   g++ -std=c++26 -O2 -Wall -Wextra -o multi_sqe multi_sqe.cpp -luring
	*   ./multi_sqe
	*
	* 五、预期输出（本机 6.18 实测的一次）
	*   一次 io_uring_submit 提交 4 个 SQE（2 个 recv + 2 个 send）
	*   第 1 个到达的 CQE：user_data=0x200（send #0）res=7
	*   第 2 个到达的 CQE：user_data=0x201（send #1）res=7
	*   第 3 个到达的 CQE：user_data=0x101（recv #1）res=7 内容="msg-one"
	*   第 4 个到达的 CQE：user_data=0x100（recv #0）res=7 内容="msg-two"
	*   校验：msg-one 收到 1 次，msg-two 收到 1 次
	*   退出（ring 与两个 fd 由 RAII 自动释放）
	*   注意：send 与 recv 的完成先后、recv #0/#1 各自拿到哪条消息，每次
	*   运行都可能不同，上例只是本机一次实测结果，绝不能写死依赖。
	*/

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <print>
#include <span>
#include <string_view>
#include <utility>

#include <sys/socket.h>
#include <unistd.h>

#include <liburing.h>

// ---------------------------------------------------------------------------
// 小工具（同 single_sqe.cpp，每个示例都自包含）
// ---------------------------------------------------------------------------
[[noreturn]] void fail(std::string_view what, int err)
{
	std::println(stderr, "{} 失败：{} (errno={})", what, std::strerror(err), err);
	std::exit(EXIT_FAILURE);
}

class UniqueFd {
public:
	UniqueFd() = default;
	explicit UniqueFd(int fd) : fd_(fd) {}
	~UniqueFd()
	{
		if (fd_ >= 0)
			::close(fd_);
	}

	UniqueFd(UniqueFd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
	UniqueFd &operator=(UniqueFd &&other) noexcept
	{
		if (this != &other) {
			if (fd_ >= 0)
				::close(fd_);
			fd_ = std::exchange(other.fd_, -1);
		}
		return *this;
	}
	UniqueFd(const UniqueFd &) = delete;
	UniqueFd &operator=(const UniqueFd &) = delete;

	int get() const { return fd_; }

private:
	int fd_ = -1;
};

class Ring {
public:
	explicit Ring(unsigned entries)
	{
		int ret = io_uring_queue_init(entries, &ring_, 0);
		if (ret < 0)
			fail("io_uring_queue_init", -ret);
		initialized_ = true;
	}
	~Ring()
	{
		if (initialized_)
			io_uring_queue_exit(&ring_);
	}
	Ring(const Ring &) = delete;
	Ring &operator=(const Ring &) = delete;

	io_uring_sqe *get_sqe()
	{
		io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
		if (sqe == nullptr)
			fail("io_uring_get_sqe（SQ 已满，应先 submit）", ENOSPC);
		return sqe;
	}

	int submit()
	{
		int ret = io_uring_submit(&ring_);
		if (ret < 0)
			fail("io_uring_submit", -ret);
		return ret;
	}

	io_uring_cqe *wait_cqe()
	{
		io_uring_cqe *cqe = nullptr;
		int ret = io_uring_wait_cqe(&ring_, &cqe);
		if (ret < 0)
			fail("io_uring_wait_cqe", -ret);
		return cqe;
	}

	void cqe_seen(io_uring_cqe *cqe)
	{
		io_uring_cqe_seen(&ring_, cqe);
	}

private:
	io_uring ring_{};
	bool initialized_ = false;
};

struct SocketPair {
	UniqueFd a;
	UniqueFd b;
};

SocketPair make_socketpair(int type)
{
	int sv[2];
	if (::socketpair(AF_UNIX, type, 0, sv) < 0)
		fail("socketpair", errno);
	return SocketPair{UniqueFd{sv[0]}, UniqueFd{sv[1]}};
}

// 运算符种类编码进 user_data 的高位，低 8 位放该种类的序号。
// 这样从 CQE 就能还原出“是哪个请求”，无需任何映射表。
enum class Kind : std::uint64_t {
	Recv = 1,
	Send = 2,
};

constexpr std::uint64_t make_user_data(Kind kind, unsigned index)
{
	return (std::to_underlying(kind) << 8) | index;
}

int main()
{
	// 两条固定长度的消息。用 std::string_view 指向静态常量，生命周期没问题。
	constexpr std::array<std::string_view, 2> kOutgoing{"msg-one", "msg-two"};
	static_assert(kOutgoing[0].size() == 7 && kOutgoing[1].size() == 7);

	// 用 SOCK_DGRAM：每条消息是一个独立报文，recv 一次正好取一条，
	// 不会像 SOCK_STREAM 那样把两条消息粘在一起。真实 TCP 是流式的，
	// 需要应用层自己分帧，这里不展开。
	auto [tx, rx] = make_socketpair(SOCK_DGRAM);

	// 两个 recv 各自的缓冲区。CQE 回来之前不能销毁或复用。
	std::array<std::array<char, 64>, kOutgoing.size()> recv_bufs{};

	Ring ring{8};

	// -----------------------------------------------------------------------
	// 连续准备 4 个 SQE。注意顺序：先 recv 后 send。
	// 内核按 SQ 顺序处理：前两个 recv 发现套接字空，会返回 -EAGAIN 并
	// 挂到 poll 上等待（socket 支持 poll，所以不会被当成失败）；接着两个
	// send 写数据，recv 被唤醒后读取。
	// -----------------------------------------------------------------------
	for (unsigned i = 0; i < kOutgoing.size(); ++i) {
		io_uring_sqe *sqe = ring.get_sqe();
		io_uring_prep_recv(sqe, rx.get(), recv_bufs[i].data(), recv_bufs[i].size(), 0);
		// prep_recv 填 opcode=IORING_OP_RECV、fd、addr、len、rw_flags=0。
		io_uring_sqe_set_data64(sqe, make_user_data(Kind::Recv, i));
		// 这里 user_data = 0x100 + i。
	}
	for (unsigned i = 0; i < kOutgoing.size(); ++i) {
		io_uring_sqe *sqe = ring.get_sqe();
		io_uring_prep_send(sqe, tx.get(), kOutgoing[i].data(), kOutgoing[i].size(), 0);
		io_uring_sqe_set_data64(sqe, make_user_data(Kind::Send, i));
		// user_data = 0x200 + i。
	}

	// 一次提交 4 个。返回值应当等于 4；如果小于 4，说明内核 SQ 暂时满，
	// 剩余 SQE 还留在用户态，需要再次 submit（liburing 会替你保留）。
	int submitted = ring.submit();
	std::println("一次 io_uring_submit 提交 {} 个 SQE（2 个 recv + 2 个 send）", submitted);

	// -----------------------------------------------------------------------
	// 收割 4 个 CQE。每个 CQE：
	//   user_data —— 我们从 SQE 带过来的编号
	//   res       —— 成功时：recv 是读到的字节数，send 是写出的字节数；
	//                失败时：-errno
	//   flags     —— 本例无特殊标志
	// 到达顺序是内核决定的：本机通常先完成 send，再完成 recv；但若有多个
	// worker 线程或不同设备，顺序会变。下面的循环只依赖 user_data。
	// -----------------------------------------------------------------------
	std::array<std::string_view, kOutgoing.size()> received{};

	for (unsigned arrival = 0; arrival < 2 * kOutgoing.size(); ++arrival) {
		io_uring_cqe *cqe = ring.wait_cqe();

		const std::uint64_t ud = cqe->user_data;
		const auto kind = static_cast<Kind>(ud >> 8);
		const unsigned index = static_cast<unsigned>(ud & 0xff);

		if (kind == Kind::Recv) {
			if (cqe->res < 0) {
				std::println("第 {} 个到达的 CQE：user_data={:#x}（recv #{}）res={}（{}）",
					arrival + 1, ud, index, cqe->res, std::strerror(-cqe->res));
			} else {
				// res 是实际读到的字节数，可能小于缓冲区长度（短读），
				// 所以一定要按 res 截取，而不是按 sizeof(buffer)。
				received[index] = std::string_view{recv_bufs[index].data(),
					static_cast<std::size_t>(cqe->res)};
				std::println("第 {} 个到达的 CQE：user_data={:#x}（recv #{}）res={} 内容=\"{}\"",
					arrival + 1, ud, index, cqe->res, received[index]);
			}
		} else {
			std::println("第 {} 个到达的 CQE：user_data={:#x}（send #{}）res={}",
				arrival + 1, ud, index, cqe->res);
			// send 返回的是写出的字节数；DGRAM 上要么整条写出，要么失败。
		}

		// 每处理完一个就立刻归还槽位。也可以攒一批再 seen，
		// 但绝不能忘记：CQ 环满（2 * SQ 容量）后内核将停止投递完成，
		// wait 会永远阻塞。
		ring.cqe_seen(cqe);
	}

	// -----------------------------------------------------------------------
	// 校验：每条消息都应该被某个 recv 收到恰好一次。
	// 注意 recv #0、recv #1 谁拿到哪条消息并不保证——内核先唤醒哪个
	// 等待者就先读哪条。这正是“靠 user_data 认领请求、靠 payload 认领
	// 数据”两个维度不能混淆的原因。
	// -----------------------------------------------------------------------
	std::println("校验：msg-one 收到 {} 次，msg-two 收到 {} 次",
		static_cast<unsigned>(received[0] == kOutgoing[0]) + static_cast<unsigned>(received[1] == kOutgoing[0]),
		static_cast<unsigned>(received[0] == kOutgoing[1]) + static_cast<unsigned>(received[1] == kOutgoing[1]));

	// 更省事的批量收割写法（本例为了逐条讲解才用循环）：
	//   unsigned head;
	//   struct io_uring_cqe *cqe;
	//   unsigned n = 0;
	//   io_uring_for_each_cqe(&ring, head, cqe) {
	//       /* 处理 cqe */ ++n;
	//   }
	//   io_uring_cq_advance(&ring, n);   // 一次性推进 head，减少原子写
	// 高吞吐场景推荐这种写法。

	std::println("退出（ring 与两个 fd 由 RAII 自动释放）");
	return 0;
}
