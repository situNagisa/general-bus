// 不在本机编译：Windows IOCP 需要 Windows SDK（winsock2.h / ws2_32.lib）与 MSVC，本机是 WSL2/Linux。
/*
	* iocp.cpp —— Windows I/O 完成端口（I/O Completion Port, IOCP）
	*
	* 一、它是什么
	* IOCP 不是一种 I/O，而是 Windows 上“异步 I/O 的完成通知总线”。你用
	* OVERLAPPED 结构提交 WSARecv / WSASend（或其他句柄的 ReadFile / WriteFile），
	* 操作在后台完成后，内核把一条“完成包”（completion packet）投递到完成端口。
	* 工作线程只需要在一个端口上调用 GetQueuedCompletionStatus(Ex) 取结果，
	* 不需要为每个 socket 开线程，也不需要 select/poll 扫描。
	*
	* 二、解决什么问题、什么时候用
	* 大量并发连接时，线程数不能随连接数增长。IOCP 的模型是：少量线程阻塞在
	* 完成端口上，内核完成一个 I/O 就唤醒一个线程去处理，天然形成线程池；
	* 同时它把“提交”和“完成”彻底解耦：提交请求的线程可以先走，完成的顺序
	* 也不保证与提交顺序一致。Windows 上的高性能服务器（HTTP、数据库、
	* 代理）基本都建立在这套机制上。
	*
	* 三、它的形态（和 Linux epoll/io_uring 的对照）
	*   - 一次 WSASend/WSARecv 提交 1 个 OVERLAPPED 请求，对应 1 个完成包；
	*     一次 GetQueuedCompletionStatusEx 可以一次取回最多 N 个完成包（批量）。
	*   - 完成包里的 lpOverlapped 就是提交时传进去的那个指针，内核原样带回。
	*     因此通常把 OVERLAPPED 放在请求结构体的第一个成员，收到完成包时
	*     把指针转回整个结构体，就知道“这是哪个请求、哪个 socket、什么缓冲区”。
	*   - 完成包不保证顺序，也不保证成功：失败的操作同样会产生完成包
	*     （lpOverlapped 非空），要在处理时用 WSAGetOverlappedResult 取错误码。
	*   - 内存由调用者提供：缓冲区、OVERLAPPED 都必须活到完成包被取出为止。
	*
	* 四、FILE_SKIP_COMPLETION_PORT_ON_SUCCESS
	* 正常情况下，即使 WSASend 同步返回成功（数据已经拷入内核），内核仍会
	* 往完成端口投一个“成功”完成包，于是应用会为一个已经处理过的操作再收一次，
	* 属于白白的唤醒和锁开销。对每个句柄调用
	*   SetFileCompletionNotificationModes(handle, FILE_SKIP_COMPLETION_PORT_ON_SUCCESS)
	* 后，凡是“同步成功”的操作都不再投完成包，API 直接返回 0；而
	* WSA_IO_PENDING（真正异步）和失败仍然会投完成包。代价是应用必须自己
	* 处理“返回 0 表示已经完成”的情况，否则会漏掉这次完成。
	* 该标志是“每个句柄”的设置，不是完成端口的设置；老的/第三方 provider
	* 可能不支持，调用失败时退回普通模式即可（本示例就是这么做的）。
	*
	* 五、条件与编译
	*   Windows Vista 及以上（GetQueuedCompletionStatusEx）；本示例用 TCP 回环
	*   自连，不需要外部网络。
	*   cl /std:c++latest /W4 /EHsc /utf-8 iocp.cpp ws2_32.lib
	*   （源码里有 #pragma comment(lib, "ws2_32.lib")，直接 cl iocp.cpp 也可以）
	*
	* 六、运行后的预期输出
	*   打印完成端口、两个 socket 的关联结果和跳过成功完成包的设置；
	*   客户端发 "hello IOCP"，服务端收到后原样回发，客户端收到回显；
	*   每次完成都打印方向、请求 id、字节数，最后打印总完成包数量。
	*/

#ifndef _WIN32
#error "iocp.cpp 只能在 Windows 上用 MSVC 编译"
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600 // GetQueuedCompletionStatusEx 需要 Vista+
#endif
#include <winsock2.h>
#include <windows.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#pragma comment(lib, "ws2_32.lib")

namespace {

/* 出错时把 WSA 错误码一起抛出；注意必须在做任何可能改变 last error 的
	* 事情之前先把它读出来。 */
[[noreturn]] void throw_wsa(std::string_view what)
{
	const int err = WSAGetLastError();
	throw std::runtime_error(std::string(what) + " 失败，WSA 错误码 " +
		std::to_string(err));
}

[[noreturn]] void throw_win32(std::string_view what)
{
	const DWORD err = GetLastError();
	throw std::runtime_error(std::string(what) + " 失败，Win32 错误码 " +
		std::to_string(err));
}

/* ------------------------------------------------------------------ */
/* RAII：WSAStartup / WSACleanup。整个进程只初始化一次。               */
/* ------------------------------------------------------------------ */
class winsock_runtime {
public:
	winsock_runtime()
	{
		WSADATA data{};
		// 请求 2.2；返回值是错误码本身，不是 WSAGetLastError。
		const int rc = WSAStartup(MAKEWORD(2, 2), &data);
		if (rc != 0)
			throw std::runtime_error("WSAStartup 失败，错误码 " + std::to_string(rc));
	}
	~winsock_runtime() { WSACleanup(); }

	winsock_runtime(const winsock_runtime &) = delete;
	winsock_runtime &operator=(const winsock_runtime &) = delete;
};

/* ------------------------------------------------------------------ */
/* RAII：SOCKET。closesocket 而不是 CloseHandle。                       */
/* ------------------------------------------------------------------ */
class unique_socket {
public:
	unique_socket() = default;
	explicit unique_socket(SOCKET s) noexcept : s_(s) {}
	~unique_socket() { reset(); }

	unique_socket(const unique_socket &) = delete;
	unique_socket &operator=(const unique_socket &) = delete;

	unique_socket(unique_socket &&other) noexcept
		: s_(std::exchange(other.s_, INVALID_SOCKET))
	{
	}

	unique_socket &operator=(unique_socket &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.s_, INVALID_SOCKET));
		return *this;
	}

	SOCKET get() const noexcept { return s_; }

	void reset(SOCKET s = INVALID_SOCKET) noexcept
	{
		if (s_ != INVALID_SOCKET)
			::closesocket(s_);
		s_ = s;
	}

private:
	SOCKET s_ = INVALID_SOCKET;
};

/* ------------------------------------------------------------------ */
/* RAII：内核 HANDLE（完成端口、事件等）。                             */
/* ------------------------------------------------------------------ */
class unique_handle {
public:
	unique_handle() = default;
	explicit unique_handle(HANDLE h) noexcept : h_(h) {}
	~unique_handle() { reset(); }

	unique_handle(const unique_handle &) = delete;
	unique_handle &operator=(const unique_handle &) = delete;

	unique_handle(unique_handle &&other) noexcept
		: h_(std::exchange(other.h_, nullptr))
	{
	}

	unique_handle &operator=(unique_handle &&other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.h_, nullptr));
		return *this;
	}

	HANDLE get() const noexcept { return h_; }
	explicit operator bool() const noexcept { return h_ != nullptr; }

	void reset(HANDLE h = nullptr) noexcept
	{
		if (h_ != nullptr && h_ != INVALID_HANDLE_VALUE)
			::CloseHandle(h_);
		h_ = h;
	}

private:
	HANDLE h_ = nullptr;
};

/* 一次异步操作的完整上下文。
	* 关键：OVERLAPPED 必须是第一个成员，这样完成包里的 lpOverlapped
	* 可以直接 static_cast 回 io_request*（标准布局 + 首成员偏移 0）。
	* 这个对象在请求挂起期间不能被移动或释放。 */
struct io_request {
	enum class kind { send, recv };

	OVERLAPPED ov{}; // 提交时传给 WSASend/WSARecv；内核完成后原样带回
	SOCKET sock = INVALID_SOCKET;
	WSABUF wsabuf{}; // 指向 buf；长度由 post_* 填
	DWORD flags = 0; // WSARecv 的 flags 在异步期间必须保持有效
	kind op = kind::send;
	int id = 0;
	std::array<char, 256> buf{}; // 收发缓冲区
};

static_assert(offsetof(io_request, ov) == 0,
	"OVERLAPPED 必须在结构体第一个成员，完成包指针才能转回 io_request");

/* 演示用的全局状态：完成端口、已处理完成包数、最后一次收到的数据。 */
struct demo_state {
	unique_handle iocp;
	int completed = 0;
	std::string last_recv;
};

/* 处理一个已经完成的请求。bytes 是实际传输的字节数。 */
void on_complete(io_request &rq, DWORD bytes, demo_state &st)
{
	if (rq.op == io_request::kind::recv) {
		const auto n = static_cast<std::size_t>(bytes);
		st.last_recv.assign(rq.buf.data(), n);
		std::printf("  [recv id=%d] %lu 字节: %.*s\n", rq.id,
			static_cast<unsigned long>(bytes), static_cast<int>(bytes),
			rq.buf.data());
	} else {
		std::printf("  [send id=%d] %lu 字节已完成\n", rq.id,
			static_cast<unsigned long>(bytes));
	}
	++st.completed;
}

/* 提交一次发送。skip_success 对应 FILE_SKIP_COMPLETION_PORT_ON_SUCCESS：
	* 如果为 true 且 WSASend 同步返回成功，则不会再有完成包，必须在这里
	* 直接调用 on_complete；否则完成包稍后由完成端口送来。 */
void post_send(SOCKET s, io_request &rq, std::string_view text,
	bool skip_success, demo_state &st)
{
	if (text.size() > rq.buf.size())
		throw std::runtime_error("post_send: 发送数据超过 256 字节的演示缓冲区");

	rq.op = io_request::kind::send;
	rq.sock = s;
	rq.wsabuf.len = static_cast<ULONG>(text.size());
	rq.wsabuf.buf = rq.buf.data();
	std::memcpy(rq.buf.data(), text.data(), text.size());

	// 复用 OVERLAPPED 前必须清零（尤其是 Internal/Offset 这些内核字段）；
	// 与 IOCP 一起用时 hEvent 不参与完成通知，保持 NULL。
	std::memset(&rq.ov, 0, sizeof rq.ov);

	DWORD sent = 0;
	const int rc = ::WSASend(s, &rq.wsabuf, 1, &sent, 0, &rq.ov, nullptr);
	if (rc == SOCKET_ERROR) {
		const int err = WSAGetLastError();
		if (err != WSA_IO_PENDING)
			throw_wsa("WSASend");
		// WSA_IO_PENDING：已排队，完成包稍后到。
		return;
	}
	// 返回 0：本次调用已经把数据交给内核（同步完成）。
	if (skip_success)
		on_complete(rq, sent, st); // 设置了跳过成功完成包，只能现在处理
	// 未设置时，内核仍会投递一个成功完成包，等 drain 去取。
}

/* 提交一次接收。注意 lpFlags 指向 rq.flags，该字段必须活到完成。 */
void post_recv(SOCKET s, io_request &rq, bool skip_success, demo_state &st)
{
	rq.op = io_request::kind::recv;
	rq.sock = s;
	rq.wsabuf.len = static_cast<ULONG>(rq.buf.size());
	rq.wsabuf.buf = rq.buf.data();
	rq.flags = 0;
	std::memset(&rq.ov, 0, sizeof rq.ov);

	DWORD received = 0;
	const int rc = ::WSARecv(s, &rq.wsabuf, 1, &received, &rq.flags,
		&rq.ov, nullptr);
	if (rc == SOCKET_ERROR) {
		const int err = WSAGetLastError();
		if (err != WSA_IO_PENDING)
			throw_wsa("WSARecv");
		return;
	}
	if (skip_success)
		on_complete(rq, received, st);
}

/* 批量收完成包，直到 st.completed 达到 target。
	* GetQueuedCompletionStatusEx 一次最多返回 entries.size() 个完成包；
	* bAlertable=FALSE 表示不处理 APC，只等 I/O 完成（这也是绝大多数服务器
	* 想要的语义；TRUE 会让线程在等待期间执行 APC 回调）。 */
void drain(demo_state &st, int target, DWORD timeout_ms)
{
	std::array<OVERLAPPED_ENTRY, 8> entries{};
	while (st.completed < target) {
		ULONG count = 0;
		if (!::GetQueuedCompletionStatusEx(st.iocp.get(), entries.data(),
				static_cast<ULONG>(entries.size()), &count, timeout_ms, FALSE)) {
			const DWORD err = GetLastError();
			if (err == WAIT_TIMEOUT)
				throw std::runtime_error("等待完成包超时（已处理 " +
					std::to_string(st.completed) + " 个）");
			throw_win32("GetQueuedCompletionStatusEx");
		}
		for (ULONG i = 0; i < count; ++i) {
			// lpOverlapped 就是提交时那个 &rq.ov；OVERLAPPED 在结构体首位，
			// 所以可以直接转回 io_request。
			auto *rq = reinterpret_cast<io_request *>(entries[i].lpOverlapped);
			// 完成包本身只给出“有这件事”，真正的结果（成功/失败/字节数）
			// 用 WSAGetOverlappedResult 读；bWait=FALSE，因为已经完成了。
			DWORD bytes = 0;
			DWORD flags = 0;
			if (!::WSAGetOverlappedResult(rq->sock, &rq->ov, &bytes, FALSE, &flags))
				throw_wsa("WSAGetOverlappedResult（该 I/O 失败）");
			on_complete(*rq, bytes, st);
		}
	}
}

/* 把监听 socket 绑到 127.0.0.1 的临时端口，返回端口号。 */
std::uint16_t bind_loopback(SOCKET s)
{
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1，不碰外部网络
	addr.sin_port = 0; // 0：让系统选一个空闲端口
	if (::bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == SOCKET_ERROR)
		throw_wsa("bind(127.0.0.1:0)");

	int len = sizeof addr;
	if (::getsockname(s, reinterpret_cast<sockaddr *>(&addr), &len) == SOCKET_ERROR)
		throw_wsa("getsockname");
	return ntohs(addr.sin_port);
}

} // namespace

int main()
{
	try {
		winsock_runtime wsa;

		demo_state st;
		// 第一个参数 INVALID_HANDLE_VALUE、第二个 NULL：创建一个新的完成端口。
		// 最后一个参数 0：允许的并发线程数交给系统（通常等于 CPU 数）。
		st.iocp.reset(::CreateIoCompletionPort(INVALID_HANDLE_VALUE, nullptr, 0, 0));
		if (!st.iocp)
			throw_win32("CreateIoCompletionPort(创建)");
		std::printf("[1] 完成端口已创建\n");

		// ---- 建立一条本机 TCP 连接：listener → client 主动连 → accept ----
		// 这一步用阻塞 API 完成，连接建立后再把两端切成异步（关联 IOCP）。
		unique_socket listener(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
		if (listener.get() == INVALID_SOCKET)
			throw_wsa("socket(listener)");

		const std::uint16_t port = bind_loopback(listener.get());
		if (::listen(listener.get(), 1) == SOCKET_ERROR)
			throw_wsa("listen");

		sockaddr_in peer{};
		peer.sin_family = AF_INET;
		peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		peer.sin_port = htons(port);

		unique_socket client(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
		if (client.get() == INVALID_SOCKET)
			throw_wsa("socket(client)");
		if (::connect(client.get(), reinterpret_cast<sockaddr *>(&peer),
				sizeof peer) == SOCKET_ERROR)
			throw_wsa("connect(127.0.0.1)");

		unique_socket server(::accept(listener.get(), nullptr, nullptr));
		if (server.get() == INVALID_SOCKET)
			throw_wsa("accept");
		listener.reset(); // 后面不再需要监听 socket
		std::printf("[2] TCP 回环连接已建立：127.0.0.1:%u\n", static_cast<unsigned>(port));

		// ---- 把两个 socket 关联到完成端口 ----
		// 第三个参数 completion key 是每个句柄的“身份证”，完成包会带回来；
		// 本示例用 1/2 区分服务端/客户端（也可以用 socket 指针）。第四个
		// 参数只对第一个关联调用有意义，0 表示由系统决定并发线程数。
		if (::CreateIoCompletionPort(reinterpret_cast<HANDLE>(server.get()),
				st.iocp.get(), 1, 0) == nullptr)
			throw_win32("CreateIoCompletionPort(server)");
		if (::CreateIoCompletionPort(reinterpret_cast<HANDLE>(client.get()),
				st.iocp.get(), 2, 0) == nullptr)
			throw_win32("CreateIoCompletionPort(client)");
		std::printf("[3] server/client 已关联到同一个完成端口\n");

		// ---- 打开“成功完成不投包” ----
		// 返回 FALSE 说明 provider/句柄不支持该模式：此时保持默认（成功也投包），
		// 代码依然正确，只是多一次唤醒。绝不能“调用失败但仍跳过完成包”。
		const bool skip_server = ::SetFileCompletionNotificationModes(
			reinterpret_cast<HANDLE>(server.get()),
			FILE_SKIP_COMPLETION_PORT_ON_SUCCESS) != FALSE;
		const bool skip_client = ::SetFileCompletionNotificationModes(
			reinterpret_cast<HANDLE>(client.get()),
			FILE_SKIP_COMPLETION_PORT_ON_SUCCESS) != FALSE;
		std::printf("[4] FILE_SKIP_COMPLETION_PORT_ON_SUCCESS: server=%s client=%s\n",
			skip_server ? "on" : "off（不支持，退回默认模式）",
			skip_client ? "on" : "off（不支持，退回默认模式）");

		// 两个请求对象必须在整个异步期间存活、地址不变。
		io_request server_rq;
		io_request client_rq;
		server_rq.id = 1;
		client_rq.id = 2;

		// ---- 第一轮：客户端发，服务端收 ----
		// 先挂接收，保证数据到达时有缓冲区；再提交发送。
		post_recv(server.get(), server_rq, skip_server, st);
		post_send(client.get(), client_rq, "hello IOCP", skip_client, st);
		std::printf("[5] 已提交 server recv / client send\n");
		drain(st, 2, 5000); // 两个完成包可能一次批量返回

		// ---- 第二轮：服务端把刚收到的内容原样回发，客户端收 ----
		// 复用 server_rq/client_rq 是安全的：上一轮已经完成（drain 保证）。
		post_send(server.get(), server_rq, st.last_recv, skip_server, st);
		post_recv(client.get(), client_rq, skip_client, st);
		std::printf("[6] 已提交 server echo send / client recv\n");
		drain(st, 4, 5000);

		std::printf("[7] 完成：共处理 %d 个完成包，回显内容 \"%s\"\n",
			st.completed, st.last_recv.c_str());
		std::printf("IOCP 演示结束。\n");
	} catch (const std::exception &e) {
		// unique_socket / unique_handle 在这里按声明的逆序析构：
		// 先关 socket，再关完成端口，最后 WSACleanup。
		std::printf("失败：%s\n", e.what());
		return 1;
	}
	return 0;
}
