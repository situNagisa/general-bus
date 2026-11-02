/*
	* single_sqe.cpp — io_uring 的最小示例：提交 1 个 SQE，等待 1 个 CQE
	*
	* 一、这是什么、解决什么问题
	* io_uring 是 Linux 5.1 起提供的异步 I/O 接口。普通的 read/recv 是同步调用：
	* 线程在内核里等到数据才返回，等的时候什么也干不了；高并发时“一个连接一个
	* 线程”又太贵。io_uring 把“提交请求”和“收割完成”拆成两个由用户态和内核
	* 共同读写的共享内存环：
	*   - SQ（Submission Queue，提交环）：应用把请求结构体 SQE 放进 SQ，
	*     一次 io_uring_enter 就能整批交给内核，省掉每个操作一次系统调用；
	*   - CQ（Completion Queue，完成环）：内核把结果结构体 CQE 放进 CQ，
	*     应用批量取走。
	* 一个线程可以同时挂起成千上万个请求，谁先完成就先处理谁，不需要一个
	* 连接配一个线程。
	*
	* 二、形态（本例）
	*   - 一次 io_uring_submit() 提交 1 个 SQE（本例是一次 recv），
	*     但接口本身允许一次提交 N 个，见 multi_sqe.cpp；
	*   - 1 个 SQE 对应 1 次内核操作，正常产生 1 个 CQE；
	*   - 对应的“线上单元”：本例只有一条 UNIX 域流式连接。recv 的返回值是
	*     字节数，流式套接字不保证一次 recv 正好取到一条完整消息，可能短读；
	*   - 数据缓冲区由应用提供（这里是 std::array<char, 64>），必须保证在
	*     CQE 被 io_uring_cqe_seen() 收割之前一直有效、不被移动或释放；
	*   - SQ/CQ 两个环以及 SQE/CQE 数组本身由内核在 io_uring_setup 时分配，
	*     liburing 通过 mmap 映射到用户态，应用只读写环的头/尾索引和环里的
	*     槽位，不拥有这些内存。
	*
	* 三、内核条件
	*   - 内核 >= 5.1（io_uring 和 IORING_OP_RECV 都是 5.1 引入）；
	*   - 不需要特殊硬件；本机 WSL2 内核 6.18 直接可跑。
	*
	* 四、编译与运行
	*   g++ -std=c++26 -O2 -Wall -Wextra -o single_sqe single_sqe.cpp -luring
	*   ./single_sqe
	*
	* 五、预期输出（本机 6.18 实测）
	*   预先写入 16 字节："hello, io_uring!"
	*   io_uring_queue_init 完成：SQ 环 8 项，CQ 环 16 项
	*   io_uring_submit 返回 1（本次交给内核的 SQE 数）
	*   收到 CQE：user_data=0x1001 res=16 flags=0x0
	*   recv 得到 16 字节："hello, io_uring!"
	*   io_uring_cqe_seen：CQ head 已推进，这个 CQE 槽位归还内核
	*   退出：queue_exit 已 unmap 两个环并关闭 ring fd
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
// 本文件自带的小工具：出错就打印原因并退出。
// io_uring 系列函数（除 submit 外）按 liburing 惯例返回“负的 errno”，
// 所以调用处要写 -ret 才能和 strerror() 对上。
// ---------------------------------------------------------------------------
[[noreturn]] void fail(std::string_view what, int err)
{
	std::println(stderr, "{} 失败：{} (errno={})", what, std::strerror(err), err);
	std::exit(EXIT_FAILURE);
}

// fd 的 RAII 包装：析构自动 close，避免示例中途 return 时泄漏。
// 移动构造把原对象置为 -1，保证同一时刻只有一个所有者。
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

// io_uring 实例的 RAII 包装：析构自动 io_uring_queue_exit。
// 注意 io_uring 结构体必须保持地址不变（内核与用户态共享它的 mmap 地址），
// 因此本类禁止拷贝，且对象要放在稳定的位置（这里是 main 的栈上）。
class Ring {
public:
	explicit Ring(unsigned entries)
	{
		// entries：期望的 SQ 槽位数，内核会向上取整到 2 的幂；
		// CQ 环容量固定是它的 2 倍（内核保证完成不会因为应用来不及收割
		// 而丢，除非溢出，见 io_uring_setup 手册）。
		// flags：0 表示默认的中断驱动模式；IORING_SETUP_SQPOLL 等模式见
		// io_uring_setup(2)，本例不需要。
		int ret = io_uring_queue_init(entries, &ring_, 0);
		if (ret < 0)
			fail("io_uring_queue_init", -ret);
		initialized_ = true;
	}
	~Ring()
	{
		// unmap SQ/CQ/SQE 内存并关闭 ring fd。之后不能再碰 ring_。
		if (initialized_)
			io_uring_queue_exit(&ring_);
	}
	Ring(const Ring &) = delete;
	Ring &operator=(const Ring &) = delete;

	// 从 SQ 环取一个空 SQE 槽位。liburing 只是返回
	// sqes[tail & ring_mask] 并把本地 tail 加一，还没有通知内核。
	// 返回 nullptr 表示 SQ 满了：应该先 io_uring_submit() 再取。
	io_uring_sqe *get_sqe()
	{
		io_uring_sqe *sqe = io_uring_get_sqe(&ring_);
		if (sqe == nullptr)
			fail("io_uring_get_sqe（SQ 已满，应先 submit）", ENOSPC);
		return sqe;
	}

	// 把本地 SQ 中未提交的 SQE 一次性交给内核（底层是 io_uring_enter）。
	// 返回值 >= 0 是“本次成功提交的个数”，不是错误码；
	// 小于准备好的个数通常意味着内核 SQ 暂时满，剩余部分要再 submit。
	int submit()
	{
		int ret = io_uring_submit(&ring_);
		if (ret < 0)
			fail("io_uring_submit", -ret);
		return ret;
	}

	// 阻塞到至少有一个 CQE。返回 0 表示拿到 cqe；负值是 -errno
	// （例如被信号打断返回 -EINTR，此时应重试）。
	io_uring_cqe *wait_cqe()
	{
		io_uring_cqe *cqe = nullptr;
		int ret = io_uring_wait_cqe(&ring_, &cqe);
		if (ret < 0)
			fail("io_uring_wait_cqe", -ret);
		return cqe;
	}

	// 告诉内核“这个 CQE 我处理完了”：推进 CQ 的 head。
	// 不调用它，CQ 满了之后内核就无法再写完成事件，wait 会永远等下去。
	void cqe_seen(io_uring_cqe *cqe)
	{
		io_uring_cqe_seen(&ring_, cqe);
	}

	const io_uring &raw() const { return ring_; }

private:
	io_uring ring_{};
	bool initialized_ = false;
};

// 一对已连接的 UNIX 域套接字：a 写，b 读，自产自销，不依赖网络。
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

int main()
{
	constexpr std::string_view kMessage = "hello, io_uring!";

	// 1. 先建立一条本机连接。sv[0] 当作“对端”，sv[1] 当作“我方”。
	auto [tx, rx] = make_socketpair(SOCK_STREAM);

	// 2. 先把数据写进 sv[0]。这样下面那个 recv 一提交就能完成；
	//    如果先 recv 后写，recv 会挂起在 poll 上，直到对端有数据。
	ssize_t written = ::write(tx.get(), kMessage.data(), kMessage.size());
	if (written < 0)
		fail("write", errno);
	std::println("预先写入 {} 字节：\"{}\"", written, kMessage);

	// 3. 初始化 io_uring。这一步做三件事：io_uring_setup 系统调用、
	//    mmap 共享环、准备 SQE 数组。8 个槽位对本例绰绰有余。
	Ring ring{8};
	std::println("io_uring_queue_init 完成：SQ 环 {} 项，CQ 环 {} 项",
		ring.raw().sq.ring_entries, ring.raw().cq.ring_entries);

	// 4. 数据缓冲区：由应用提供，生命周期必须覆盖到 CQE 被收割。
	//    用 std::array 放在栈上最直观；高并发时通常用内存池/ring buffer。
	std::array<char, 64> buffer{};
	std::span<char> target{buffer};

	// 5. 取一个 SQE。SQE 是 64 字节的定长结构体，字段有：
	//      opcode   操作码，如 IORING_OP_RECV
	//      flags    IOSQE_* 标志位（本例 0）
	//      ioprio   I/O 优先级（本例 0）
	//      fd       目标文件描述符
	//      off/addr2 文件内偏移（recv 不用，为 0）
	//      addr     用户缓冲区地址
	//      len      缓冲区长度
	//      rw_flags 传给 recv 的 flags（MSG_*）
	//      user_data 应用私有值，完成时原样出现在 CQE 里
	//    io_uring_prep_recv() 就是把这些字段一次填好，等价于手写：
	//      sqe->opcode = IORING_OP_RECV; sqe->fd = rx.get();
	//      sqe->addr = target.data(); sqe->len = target.size();
	//      sqe->rw_flags = 0; sqe->off = 0;
	io_uring_sqe *sqe = ring.get_sqe();
	io_uring_prep_recv(sqe, rx.get(), target.data(), target.size(), 0);

	// 6. user_data 是应用与内核之间的“行李牌”。一个线程同时挂起多个
	//    请求时，CQE 回来只有这个值能告诉你它对应哪个请求。
	//    这里随便填一个；multi_sqe.cpp 会用编码后的编号。
	//    io_uring_sqe_set_data64 只是写 sqe->user_data，64 位原样返回。
	constexpr std::uint64_t kUserData = 0x1001;
	io_uring_sqe_set_data64(sqe, kUserData);

	// 7. 提交。注意：io_uring_submit 之后 SQE 就归内核所有，
	//    用户态不能再修改它，直到对应的 CQE 被 seen（复用槽位周期
	//    由内核的 tail 推进保证）。返回值是本次提交的个数。
	int submitted = ring.submit();
	std::println("io_uring_submit 返回 {}（本次交给内核的 SQE 数）", submitted);

	// 8. 等待完成。CQE 是 16 字节结构体：
	//      user_data 就是 SQE 里那个值
	//      res       >= 0 表示成功（recv 时是读到的字节数），
	//                < 0 表示失败，绝对值是 errno（例如 -ECONNRESET）
	//      flags     IORING_CQE_F_*，本例没有特殊标志，为 0
	struct io_uring_cqe *cqe = ring.wait_cqe();
	std::println("收到 CQE：user_data={:#x} res={} flags={:#x}",
		cqe->user_data, cqe->res, cqe->flags);

	if (cqe->res < 0) {
		// 失败时 res = -errno。注意先把 CQE seen 掉再退出，
		// 虽然本例要退出，但养成“每个 CQE 都必须 seen”的习惯。
		int err = -cqe->res;
		std::println(stderr, "recv 失败：{}", std::strerror(err));
		ring.cqe_seen(cqe);
		return EXIT_FAILURE;
	}
	if (cqe->res == 0)
		std::println("（res == 0 表示对端已关闭，是 EOF，不是错误）");

	std::string_view received{buffer.data(), static_cast<std::size_t>(cqe->res)};
	std::println("recv 得到 {} 字节：\"{}\"", cqe->res, received);

	// 9. 归还 CQE。CQ 环是一个环形缓冲：内核写 tail，应用写 head；
	//    seen 就是在推进 head。漏掉它 CQ 迟早被占满。
	ring.cqe_seen(cqe);
	std::println("io_uring_cqe_seen：CQ head 已推进，这个 CQE 槽位归还内核");

	// 10. 退出。ring 的析构会 queue_exit；tx/rx 的析构会 close。
	//     顺序无所谓，但缓冲区必须在 CQE seen 之后才能释放——
	//     本例 buffer 是栈变量，return 时自然满足。
	std::println("退出：queue_exit 已 unmap 两个环并关闭 ring fd");
	return 0;
}
