/*
	sendfile_splice.cpp —— sendfile（文件→socket）与 splice（经 pipe 搬 fd）

	一、这是什么
	sendfile(out_fd, in_fd, offset, count)：把 in_fd（必须是支持 mmap 的
	文件）里的数据直接送进 out_fd（socket 或普通文件），数据不经过
	用户态内存。它是“文件服务器发文件”的专用捷径。
	splice(fd_in, off_in, fd_out, off_out, len, flags)：在两个 fd 之间
	搬运数据，但要求至少一端是 pipe。想做到“socket→socket”就得
	借一个 pipe 中转：socket→pipe、pipe→socket，各调一次。
	两者都避免把数据复制到用户缓冲，区别是 sendfile 是文件→socket 的
	一条龙，splice 是通用搬运且必须借 pipe。

	二、解决什么问题 / 什么时候用
	静态文件服务（HTTP 发文件）、代理转发（socket→socket）、日志/备份
	搬运。省掉“read 到用户缓冲再 write”的一次内核→用户、一次用户→内核
	拷贝，并且省掉用户态缓冲的内存带宽。

	三、形态
	- sendfile：一次调用交 0 段用户内存；线上单元由 TCP 自己按 MSS 切；
	提交 1 次、完成 1 次（同步返回已写入 out_fd 的字节数，可能短写）；
	内存来自页缓存（page cache），由内核管理，用户不提供。
	- splice：一次调用搬 len 字节进/出 pipe；pipe 是内核里的环形缓冲
	（默认容量 64 KiB，受 /proc/sys/fs/pipe-max-size 限制），一次最多
	搬一个 pipe 容量的数据。同步返回实际搬运字节数，可能短搬。
	内存是页缓存页/skb 页的引用，仍由内核管理。
	- 两者都是“不做用户态拷贝”，但不保证网卡 DMA 也不拷贝：驱动可能
	仍要拷一次。不要把它们当成“绝对零拷贝”的保证。

	四、需要的条件
	- Linux。sendfile 自 2.2 起有；2.6.33 起 out_fd 可以是任意文件或
	socket（更早只能是 socket）。in_fd 不能是 socket/pipe（EINVAL）。
	- splice 自 2.6.17 起；至少一端必须是 pipe，否则 EINVAL。
	- 单次 sendfile 的 count 上限是 0x7ffff000（MAX_RW_COUNT）；要发的
	文件更大就循环调用。
	- 无特殊权限。

	五、编译与运行
	g++ -std=c++26 -O2 -Wall -Wextra -o /tmp/native_sendfile_splice sendfile_splice.cpp -pthread
	/tmp/native_sendfile_splice
	（用到了 std::thread 做接收端；glibc 2.34 以后 pthread 已并入 libc，
	不加 -pthread 可能也能链接，但显式加上最稳妥。）

	六、预期输出
	== 1. sendfile：1 MiB 临时文件 -> TCP 回环 socket ==
	发送 1048576 字节，文件偏移推进到 1048576（文件位置未动：0）
	接收端校验和一致
	== 2. splice：同一文件 -> pipe -> TCP 回环 socket ==
	直接 file->socket 调 splice：失败 EINVAL（符合预期：没有 pipe）
	经 pipe 搬运 1048576 字节，接收端校验和一致
	== 3. splice 中转 socket->pipe->socket（socketpair） ==
	中转 65536 字节，内容一致
*/

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>
#include <thread>
#include <utility>

namespace {

/*
	fd 的 RAII 包装：析构 close，禁止拷贝，允许移动。
*/
class Fd {
public:
	explicit Fd(int fd = -1) noexcept : fd_(fd) {}
	Fd(const Fd &) = delete;
	Fd &operator=(const Fd &) = delete;
	Fd(Fd &&other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
	Fd &operator=(Fd &&other) noexcept
	{
		if (this != &other) {
			reset();
			fd_ = std::exchange(other.fd_, -1);
		}
		return *this;
	}
	~Fd() { reset(); }

	[[nodiscard]] int get() const noexcept { return fd_; }
	explicit operator bool() const noexcept { return fd_ >= 0; }

private:
	void reset() noexcept
	{
		if (fd_ >= 0)
			::close(fd_);
		fd_ = -1;
	}

	int fd_;
};

[[noreturn]] void die(const char *what)
{
	std::fprintf(stderr, "错误：%s: %s\n", what, std::strerror(errno));
	std::exit(EXIT_FAILURE);
}

void check(bool ok, const char *what)
{
	if (!ok)
		die(what);
}

void check(const Fd &fd, const char *what)
{
	if (!fd)
		die(what);
}

constexpr std::size_t kFileBytes = 1024 * 1024;	// 1 MiB，超过 64 KiB 的 pipe 容量
constexpr std::size_t kRelayBytes = 64 * 1024;	// 正好一个默认 pipe 容量
constexpr std::uint64_t kFnvOffset = 1469598103934665603ull;

/*
	确定性的伪随机模式：同一个下标永远得到同一个字节，便于收发两端校验。
*/
std::byte pattern_byte(std::size_t i)
{
	return static_cast<std::byte>((i * 131 + 7) & 0xff);
}

std::uint64_t fnv1a(std::span<const std::byte> data, std::uint64_t h = kFnvOffset)
{
	for (const std::byte b : data) {
		h ^= static_cast<std::uint8_t>(b);
		h *= 1099511628211ull;
	}
	return h;
}

/*
	临时文件：mkstemp 建文件，析构时 close + unlink（不留下垃圾）。
*/
class TempFile {
public:
	TempFile()
	{
		std::array<char, 64> tmpl{};
		std::snprintf(tmpl.data(), tmpl.size(), "/tmp/ibus_sendfile_XXXXXX");
		const int fd = ::mkstemp(tmpl.data());
		check(fd >= 0, "mkstemp");
		// 建好就 unlink：fd 仍然有效，但 /tmp 里不再有名字。
		// 这样即使程序中途 exit（本示例的 die 就是直接 exit），也不会留下垃圾。
		check(::unlink(tmpl.data()) == 0, "unlink(temp)");
		fd_ = Fd{fd};
	}
	TempFile(const TempFile &) = delete;
	TempFile &operator=(const TempFile &) = delete;

	[[nodiscard]] int get() const noexcept { return fd_.get(); }

private:
	Fd fd_;
};

/*
	只管写：把 span 里的数据完整写进 fd。用于往临时文件里填内容。
*/
void write_all(int fd, std::span<const std::byte> data)
{
	std::size_t off = 0;
	while (off < data.size()) {
		const ssize_t n = ::write(fd, data.data() + off, data.size() - off);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			die("write");
		}
		off += static_cast<std::size_t>(n);
	}
}

/*
	接收端：从 fd 读满 total 字节并返回校验和。
	放在独立线程里跑，主线程才能安心做阻塞的 sendfile/splice。
*/
std::uint64_t recv_hash_exact(int fd, std::size_t total)
{
	std::array<std::byte, 64 * 1024> buf{};
	std::size_t got = 0;
	std::uint64_t h = kFnvOffset;
	while (got < total) {
		const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			die("recv");
		}
		if (n == 0) {
			std::fprintf(stderr, "接收端提前 EOF：%zu / %zu\n", got, total);
			std::exit(EXIT_FAILURE);
		}
		h = fnv1a({buf.data(), static_cast<std::size_t>(n)}, h);
		got += static_cast<std::size_t>(n);
	}
	return h;
}

/*
	一对 TCP 回环连接：tx 用来发，rx 用来收。
	端口交给内核选（bind 到 0），不会和别的程序冲突。
*/
struct TcpPair {
	Fd tx;
	Fd rx;
};

TcpPair make_tcp_pair()
{
	Fd listener{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(listener, "socket(listener)");

	const int one = 1;
	check(::setsockopt(listener.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof one) == 0,
		"setsockopt(SO_REUSEADDR)");

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;
	check(::bind(listener.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "bind");

	socklen_t len = sizeof addr;
	check(::getsockname(listener.get(), reinterpret_cast<sockaddr *>(&addr), &len) == 0,
		"getsockname");
	check(::listen(listener.get(), 1) == 0, "listen");

	Fd tx{::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0)};
	check(tx, "socket(tx)");
	check(::connect(tx.get(), reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0, "connect");

	Fd rx{::accept4(listener.get(), nullptr, nullptr, SOCK_CLOEXEC)};
	check(rx, "accept4");
	return {std::move(tx), std::move(rx)};
}

/*
	一对 AF_UNIX 流式 socket，用于演示 socket→pipe→socket 中转。
*/
struct UnixPair {
	Fd a;
	Fd b;
};

UnixPair make_unix_pair()
{
	int sv[2] = {-1, -1};
	check(::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv) == 0, "socketpair");
	return {Fd{sv[0]}, Fd{sv[1]}};
}

/*
	pipe：一个读端、一个写端。splice 至少需要其中一端。
*/
struct Pipe {
	Fd rd;
	Fd wr;
};

Pipe make_pipe()
{
	int p[2] = {-1, -1};
	// O_CLOEXEC：exec 后不把 pipe fd 泄漏给子进程。
	check(::pipe2(p, O_CLOEXEC) == 0, "pipe2");
	return {Fd{p[0]}, Fd{p[1]}};
}

/*
	把文件内容填满并返回预期校验和。写完后文件位置停在末尾。
*/
std::uint64_t fill_temp_file(int fd)
{
	std::array<std::byte, 64 * 1024> buf{};
	std::size_t written = 0;
	std::uint64_t h = kFnvOffset;

	while (written < kFileBytes) {
		const std::size_t n = std::min(buf.size(), kFileBytes - written);
		for (std::size_t i = 0; i < n; ++i)
			buf[i] = pattern_byte(written + i);
		const std::span<const std::byte> part{buf.data(), n};
		write_all(fd, part);
		h = fnv1a(part, h);
		written += n;
	}
	return h;
}

/*
	演示一：sendfile(file -> TCP socket)。
	注意 offset 参数：传 &off 时用 off 作为读位置，并更新 *off，但不改文件
	自身的文件位置；传 nullptr 时用并推进文件位置。
*/
void demo_sendfile()
{
	std::puts("== 1. sendfile：1 MiB 临时文件 -> TCP 回环 socket ==");

	TempFile file;
	const std::uint64_t expected = fill_temp_file(file.get());
	// 填完后文件位置在末尾。先确认大小，再回到开头，方便最后检查
	// “显式 offset 没有推进文件位置”。
	const off_t file_pos_after_write = ::lseek(file.get(), 0, SEEK_CUR);
	check(file_pos_after_write == static_cast<off_t>(kFileBytes), "文件大小");
	check(::lseek(file.get(), 0, SEEK_SET) == 0, "lseek(file, 0)");

	TcpPair pair = make_tcp_pair();

	// 接收端线程：读满 kFileBytes 就退出。必须先启动，否则 send 会被
	// 回环 socket 的发送缓冲堵住（发送端和接收端在同一个进程里）。
	std::uint64_t recv_hash = 0;
	std::thread rx([&] { recv_hash = recv_hash_exact(pair.rx.get(), kFileBytes); });

	off_t off = 0;		// 从文件开头读
	std::size_t sent = 0;
	while (sent < kFileBytes) {
		// 单次 sendfile 的 count 上限是 0x7ffff000；这里 1MiB 用不到，
		// 但真实代码要按剩余量分批。
		const ssize_t n = ::sendfile(pair.tx.get(), file.get(), &off,
			kFileBytes - sent);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			die("sendfile");
		}
		// 返回 0 表示 in_fd 已到 EOF（offset 已到文件末尾）。
		if (n == 0)
			break;
		sent += static_cast<std::size_t>(n);
	}

	rx.join();

	const off_t pos = ::lseek(file.get(), 0, SEEK_CUR);
	std::printf("发送 %zu 字节，文件偏移推进到 %lld（文件位置未动：%lld）\n", sent,
		static_cast<long long>(off), static_cast<long long>(pos));
	check(sent == kFileBytes, "sendfile 发送字节数");
	check(off == static_cast<off_t>(kFileBytes), "sendfile 更新 offset");
	check(pos == 0, "显式 offset 不改变文件位置");
	std::printf("接收端校验和%s\n", recv_hash == expected ? "一致" : "不一致");
	check(recv_hash == expected, "sendfile 数据校验");
}

/*
	演示二：splice(file -> pipe -> TCP socket)。
	先证明 file->socket 直连不行，再走 pipe 中转。
*/
void demo_splice_file()
{
	std::puts("== 2. splice：同一文件 -> pipe -> TCP 回环 socket ==");

	TempFile file;
	const std::uint64_t expected = fill_temp_file(file.get());

	// 用一个一次性的 socketpair 验证“没有 pipe 就 EINVAL”，避免污染
	// 后面真正用于传输的连接。
	{
		UnixPair throwaway = make_unix_pair();
		off_t probe_off = 0;
		errno = 0;
		const ssize_t bad = ::splice(file.get(), &probe_off, throwaway.a.get(), nullptr,
			4096, 0);
		const int saved = errno;
		if (bad < 0 && saved == EINVAL) {
			std::puts("直接 file->socket 调 splice：失败 EINVAL（符合预期：没有 pipe）");
		} else if (bad >= 0) {
			// 万一某内核允许了，也不致命，只是说明规则变了。
			std::printf("直接 file->socket 调 splice 意外成功：%zd 字节\n", bad);
		} else {
			std::printf("直接 file->socket 调 splice 失败，errno=%d（预期 EINVAL）\n",
				saved);
		}
	}

	TcpPair pair = make_tcp_pair();
	Pipe pipe = make_pipe();

	std::uint64_t recv_hash = 0;
	std::thread rx([&] { recv_hash = recv_hash_exact(pair.rx.get(), kFileBytes); });

	off_t off = 0;
	std::size_t sent = 0;
	while (sent < kFileBytes) {
		// 第一步：文件 -> pipe。一次最多搬一个 pipe 容量的数据（默认
		// 64 KiB），所以外层按剩余量循环。off 是文件的读位置，与文件
		// 自身的文件位置无关（传 nullptr 才会推进文件位置）。
		const std::size_t want = std::min<std::size_t>(64 * 1024, kFileBytes - sent);
		ssize_t in = ::splice(file.get(), &off, pipe.wr.get(), nullptr, want,
			SPLICE_F_MOVE);
		if (in < 0) {
			if (errno == EINTR)
				continue;
			die("splice(file -> pipe)");
		}
		if (in == 0)
			break;	// 文件读完

		// 第二步：pipe -> socket。必须把刚才搬进 pipe 的字节全部搬走，
		// 否则 pipe 满后下一次 splice 会阻塞。
		// SPLICE_F_MORE 是给 TCP 的提示（等价于 send 的 MSG_MORE），表示
		// 后面还有数据，别急着发小包；它只是提示，不影响正确性。
		// SPLICE_F_MOVE 是“尽量搬页而不是复制”的提示，现代内核可能忽略
		// （未核对具体版本）。
		std::size_t moved = 0;
		while (moved < static_cast<std::size_t>(in)) {
			const ssize_t out = ::splice(pipe.rd.get(), nullptr, pair.tx.get(), nullptr,
				static_cast<std::size_t>(in) - moved, SPLICE_F_MOVE | SPLICE_F_MORE);
			if (out < 0) {
				if (errno == EINTR)
					continue;
				die("splice(pipe -> socket)");
			}
			if (out == 0) {
				std::fputs("splice(pipe -> socket) 意外返回 0\n", stderr);
				std::exit(EXIT_FAILURE);
			}
			moved += static_cast<std::size_t>(out);
		}
		sent += moved;
	}

	rx.join();

	std::printf("经 pipe 搬运 %zu 字节，接收端校验和%s\n", sent,
		recv_hash == expected ? "一致" : "不一致");
	check(sent == kFileBytes, "splice 发送字节数");
	check(recv_hash == expected, "splice 数据校验");
}

/*
	演示三：splice 在 socket 之间中转 —— socket -> pipe -> socket。
	sendfile 只能从文件出发；两个 socket 之间要搬数据就得借 pipe。
*/
void demo_splice_relay()
{
	std::puts("== 3. splice 中转 socket->pipe->socket（socketpair） ==");

	UnixPair src = make_unix_pair();
	UnixPair dst = make_unix_pair();
	Pipe pipe = make_pipe();

	// 造一段数据写进 src.a。64 KiB 小于 AF_UNIX 默认发送缓冲，不会阻塞。
	std::array<std::byte, kRelayBytes> data{};
	for (std::size_t i = 0; i < data.size(); ++i)
		data[i] = pattern_byte(i);
	const std::uint64_t expected = fnv1a(data);
	write_all(src.a.get(), data);

	// src.b -> pipe -> dst.a，全程不经过用户内存。
	// 说明：src.b 和 dst.a 都是 socket，不能直接 splice，pipe 是必需的中间站。
	std::size_t relayed = 0;
	while (relayed < kRelayBytes) {
		const ssize_t in = ::splice(src.b.get(), nullptr, pipe.wr.get(), nullptr,
			kRelayBytes - relayed, 0);
		if (in < 0) {
			if (errno == EINTR)
				continue;
			die("splice(socket -> pipe)");
		}
		if (in == 0)
			break;

		std::size_t moved = 0;
		while (moved < static_cast<std::size_t>(in)) {
			const ssize_t out = ::splice(pipe.rd.get(), nullptr, dst.a.get(), nullptr,
				static_cast<std::size_t>(in) - moved, SPLICE_F_MORE);
			if (out < 0) {
				if (errno == EINTR)
					continue;
				die("splice(pipe -> socket)");
			}
			moved += static_cast<std::size_t>(out);
		}
		relayed += moved;
	}

	const std::uint64_t got = recv_hash_exact(dst.b.get(), kRelayBytes);
	std::printf("中转 %zu 字节，内容%s\n", relayed, got == expected ? "一致" : "不一致");
	check(relayed == kRelayBytes, "中转字节数");
	check(got == expected, "中转数据校验");

	/*
		常见坑：
		1. sendfile 的 in_fd 是 socket/pipe：EINVAL。in_fd 必须是文件
		（能 mmap 的对象）；反向“socket→文件”要用 splice 或普通读写。
		2. 文件很大时忘掉单次上限 0x7ffff000：必须循环，且每轮更新 offset。
		3. 把 offset 参数传错：传 &off 不改文件位置，传 nullptr 会推进
		文件位置；混用会导致“文件莫名从中间开始发”。
		4. splice 一端必须是 pipe；pipe 会满（默认 64 KiB），所以必须
		按“搬进多少就搬出多少”配对，否则死锁。
		5. splice 的返回值可能小于请求长度；网络 socket 上短写很常见，
		必须循环到搬完。
		6. splice/file 发送与 TCP_CORK/Nagle 交互会影响小包合并；需要
		精确控制分界时用 SPLICE_F_MORE / TCP_CORK。
	*/
}

} // namespace

int main()
{
	demo_sendfile();
	std::puts("");
	demo_splice_file();
	std::puts("");
	demo_splice_relay();
	return 0;
}
