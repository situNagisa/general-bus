// 不在本机编译：Windows Registered I/O 需要 Windows SDK（mswsock.h / ws2_32.lib）与 MSVC，本机是 WSL2/Linux。
/*
	* rio.cpp —— Windows Registered I/O（RIO，注册 I/O）
	*
	* 一、它是什么
	* RIO 是 Winsock 之上的一层“注册制”异步 I/O 扩展（Windows 8 起提供）。
	* 它把两件事从每次调用的开销里拿掉：
	*   1. 缓冲区先 RIORegisterBuffer 注册一次，内核把页锁定（lock）并分配一个
	*      RIO_BUFFERID，之后每次收发只传 {BufferId, Offset, Length} 三元组，
	*      不再每次调用都验证/锁定用户指针；
	*   2. 完成结果不进内核对象队列，而是由 RIOCreateCompletionQueue 创建的
	*      完成队列（CQ）保存，RIODequeueCompletion 一次取回一批 RIORESULT。
	* 目标是让 UDP/TCP 的小包收发接近“用户态轮询”的开销，同时仍走内核协议栈。
	*
	* 二、解决什么问题、什么时候用
	* 普通 overlapped Winsock（IOCP）每次 WSASend/WSARecv 都要经过一次内核
	* 转换、检查缓冲区、分配完成包；几十万 PPS 时这些固定开销很显著。
	* RIO 把缓冲区注册、请求队列（RQ）和完成队列都预先建好，提交与收割都变成
	* 对已注册结构的批量操作。适合：Windows 上的 DNS/游戏/行情/代理等小包
	* 高 PPS 服务。若包少而大，普通 IOCP 更简单，RIO 的注册和调参不划算。
	* 注意：RIO 需要 Winsock provider 支持（用 WSAIoctl 探测），并非所有
	* 网卡/协议栈实现都支持；UDP 上最常见。
	*
	* 三、它的形态
	*   - RIORegisterBuffer 一次注册一整块内存，之后所有收发都在这块内存里
	*     用 offset/length 切片；注册期间这块内存被锁页，不能释放。
	*   - RIOCreateRequestQueue 给一个 socket 建 RQ，同时规定“最多几个未完成
	*     接收/发送、每次最多几个缓冲区”；超过上限的提交会失败（WSAENOBUFS）。
	*   - 一次 RIOReceive/RIOSend 提交 1 个请求，对应 1 个数据报（UDP）或
	*     1 段流数据（TCP）；请求完成时在 CQ 里产生 1 条 RIORESULT。
	*     一次 RIODequeueCompletion 可以批量取回 N 条结果。
	*   - 完成队列的通知用 RIONotify 注册：事件（RIO_EVENT_COMPLETION）或
	*     IOCP（RIO_IOCP_COMPLETION）。RIONotify 是一次性的，每次被通知后
	*     要重新调用；上一次通知还没完成时再调会返回 WSAEALREADY。
	*   - 内存由用户提供（注册缓冲区），CQ/RQ 由 provider 分配，句柄由用户持有。
	*
	* 四、关于“RIO_MSG_DEFER + RIOCommitSends”
	* 网上常能看到这种说法，但 RIOCommitSends 并不是 Winsock 的 API：
	* RIO_EXTENSION_FUNCTION_TABLE 里只有 RIOReceive/RIOSend/RIOReceiveEx/
	* RIOSendEx 等（见 mswsock.h）。RIO_MSG_DEFER 的真正提交方式是：
	* 再调用一次 RIOSend，flags 只填 RIO_MSG_COMMIT_ONLY，pData=NULL、
	* DataBufferCount=0、RequestContext=NULL。MSDN 的 LPFN_RIOSEND 文档明确
	* 写了这一点（“Previous requests added with RIO_MSG_DEFER flag will be
	* committed”）。本示例用 DEFER 连发 4 个包，再用 COMMIT_ONLY 一次性提交，
	* 演示批量提交的形态。
	*
	* 五、条件与编译
	*   Windows 8 / Server 2012 及以上；provider 支持 RIO。
	*   cl /std:c++latest /W4 /EHsc /utf-8 rio.cpp ws2_32.lib
	*
	* 六、运行后的预期输出
	*   打印函数表获取结果、CQ/RQ/缓冲区注册结果；向 127.0.0.1 的另一个 UDP
	*   socket 连发 4 个数据报（DEFER 后 COMMIT_ONLY），再批量 dequeue，
	*   打印每条结果的 Status/BytesTransferred/RequestContext 和收到的内容。
	*/

#ifndef _WIN32
#error "rio.cpp 只能在 Windows 上用 MSVC 编译"
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602 // RIO 自 Windows 8 起提供
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mswsock.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

namespace {

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
/* RAII：WSAStartup / WSACleanup。                                     */
/* ------------------------------------------------------------------ */
class winsock_runtime {
public:
	winsock_runtime()
	{
		WSADATA data{};
		const int rc = WSAStartup(MAKEWORD(2, 2), &data);
		if (rc != 0)
			throw std::runtime_error("WSAStartup 失败，错误码 " + std::to_string(rc));
	}
	~winsock_runtime() { WSACleanup(); }

	winsock_runtime(const winsock_runtime &) = delete;
	winsock_runtime &operator=(const winsock_runtime &) = delete;
};

/* ------------------------------------------------------------------ */
/* RAII：SOCKET。                                                      */
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
/* RAII：WSAEVENT / HANDLE。                                          */
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

	void reset(HANDLE h = nullptr) noexcept
	{
		if (h_ != nullptr && h_ != INVALID_HANDLE_VALUE)
			::CloseHandle(h_);
		h_ = h;
	}

private:
	HANDLE h_ = nullptr;
};

/* ------------------------------------------------------------------ */
/* 从 provider 取 RIO 函数表。参数必须是已创建、尚未关闭的 socket。     */
/* ------------------------------------------------------------------ */
RIO_EXTENSION_FUNCTION_TABLE query_rio_table(SOCKET s)
{
	RIO_EXTENSION_FUNCTION_TABLE table{};
	// cbSize 是函数表结构自身大小；按文档惯例先填好再交给 WSAIoctl。
	table.cbSize = sizeof table;

	GUID guid = WSAID_MULTIPLE_RIO; // 标识 RIO 扩展函数集的 GUID
	DWORD bytes = 0;
	if (::WSAIoctl(s, SIO_GET_MULTIPLE_EXTENSION_FUNCTION_POINTER,
			&guid, sizeof guid,
			&table, sizeof table,
			&bytes, nullptr, nullptr) == SOCKET_ERROR) {
		throw_wsa("WSAIoctl(SIO_GET_MULTIPLE_EXTENSION_FUNCTION_POINTER)");
	}
	// 只检查两个关键函数，避免 provider 返回一张空表却继续用下去。
	if (table.RIOSend == nullptr || table.RIOReceive == nullptr ||
		table.RIOCreateCompletionQueue == nullptr ||
		table.RIOCreateRequestQueue == nullptr ||
		table.RIORegisterBuffer == nullptr ||
		table.RIODequeueCompletion == nullptr ||
		table.RIONotify == nullptr)
		throw std::runtime_error("该 Winsock provider 不支持 RIO");
	return table;
}

/* RIO 完成队列的 RAII。销毁时会关闭 CQ，因此必须保证在它之前 socket 已经
	* 关闭（RQ 随 closesocket 释放）、所有请求都已完成。 */
struct rio_cq_guard {
	RIO_EXTENSION_FUNCTION_TABLE table{};
	RIO_CQ cq = RIO_INVALID_CQ;

	rio_cq_guard() = default;
	rio_cq_guard(const rio_cq_guard &) = delete;
	rio_cq_guard &operator=(const rio_cq_guard &) = delete;

	~rio_cq_guard()
	{
		if (cq != RIO_INVALID_CQ)
			table.RIOCloseCompletionQueue(cq);
	}

	void reset(const RIO_EXTENSION_FUNCTION_TABLE &t, RIO_CQ c)
	{
		table = t;
		cq = c;
	}
};

/* 注册缓冲区的 RAII。销毁时注销；必须保证此时已无未完成请求，且内存本身
	* 还没被释放（声明的顺序保证析构时缓冲区仍然有效）。 */
struct rio_buffer_guard {
	RIO_EXTENSION_FUNCTION_TABLE table{};
	RIO_BUFFERID id = RIO_INVALID_BUFFERID;

	rio_buffer_guard() = default;
	rio_buffer_guard(const rio_buffer_guard &) = delete;
	rio_buffer_guard &operator=(const rio_buffer_guard &) = delete;

	~rio_buffer_guard()
	{
		if (id != RIO_INVALID_BUFFERID)
			table.RIODeregisterBuffer(id);
	}

	void reset(const RIO_EXTENSION_FUNCTION_TABLE &t, RIO_BUFFERID b)
	{
		table = t;
		id = b;
	}
};

/* 接收区/发送区在注册缓冲区里的布局。 */
constexpr ULONG kMaxDatagram = 256; // 每个数据报最大长度，也是每个接收缓冲区大小
constexpr ULONG kSendArea = 0; // 发送数据从 offset 0 开始，每包占 kMaxDatagram
constexpr ULONG kRecvArea = 8192; // 接收区从 8 KiB 开始，避开发送区
constexpr int kMessages = 4;

std::uint16_t bind_loopback(SOCKET s)
{
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0; // 系统分配空闲端口
	if (::bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == SOCKET_ERROR)
		throw_wsa("bind(127.0.0.1:0)");

	int len = sizeof addr;
	if (::getsockname(s, reinterpret_cast<sockaddr *>(&addr), &len) == SOCKET_ERROR)
		throw_wsa("getsockname");
	return ntohs(addr.sin_port);
}

/* 建一个 RIO 专用 socket：必须带 WSA_FLAG_REGISTERED_IO，
	* 否则取函数表/建 RQ 会失败。WSA_FLAG_OVERLAPPED 是异步的默认标志。 */
SOCKET make_rio_socket()
{
	SOCKET s = ::WSASocket(AF_INET, SOCK_DGRAM, IPPROTO_UDP, nullptr, 0,
		WSA_FLAG_OVERLAPPED | WSA_FLAG_REGISTERED_IO);
	if (s == INVALID_SOCKET)
		throw_wsa("WSASocket(WSA_FLAG_REGISTERED_IO)");
	return s;
}

} // namespace

int main()
{
	try {
		winsock_runtime wsa;

		// 缓冲区要活到注销为止；声明顺序保证 guards 先析构、buffer 后释放。
		std::vector<char> buffer(64 * 1024);
		std::memset(buffer.data(), 0, buffer.size());

		// 声明顺序决定析构顺序（逆序）：event 最后关，socket 最先关，
		// 然后注销缓冲区，最后关闭 CQ。正好是 RIO 要求的释放顺序。
		unique_handle event;
		rio_cq_guard cq_guard;
		rio_buffer_guard buf_guard;
		unique_socket recv_sock;
		unique_socket send_sock;

		// ---- 建两个 UDP socket 并连到回环 ----
		recv_sock.reset(make_rio_socket());
		const std::uint16_t port = bind_loopback(recv_sock.get());

		send_sock.reset(make_rio_socket());
		sockaddr_in peer{};
		peer.sin_family = AF_INET;
		peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		peer.sin_port = htons(port);
		// RIOSend 没有“目的地”参数；对 UDP 先把 socket connect 到对端，
		// 这样它才是一个“connected UDP socket”。若不想 connect，
		// 要用 RIOSendEx 并在 pRemoteAddress 里给地址。
		if (::connect(send_sock.get(), reinterpret_cast<sockaddr *>(&peer),
				sizeof peer) == SOCKET_ERROR)
			throw_wsa("connect(RIO UDP)");
		std::printf("[1] UDP 回环：接收端 127.0.0.1:%u，发送端已 connect\n",
			static_cast<unsigned>(port));

		// ---- 取 RIO 函数表 ----
		// 表是按 provider 全局的，拿任意一个 RIO socket 问一次即可。
		const RIO_EXTENSION_FUNCTION_TABLE rio = query_rio_table(recv_sock.get());
		std::printf("[2] RIO 函数表已取得（RIOSend/RIOReceive/CQ/RQ/缓冲区接口齐全）\n");

		// ---- 建完成队列 ----
		// RIO_NOTIFICATION_COMPLETION 决定“CQ 非空时怎么通知我”：
		//   RIO_EVENT_COMPLETION：SetEvent 到 EventHandle，配 WSAWaitForMultipleEvents；
		//   RIO_IOCP_COMPLETION：投递到 IOCP，和普通 overlapped I/O 混在一起。
		// NotifyReset=TRUE 表示事件被通知后自动复位，省去 WSAResetEvent。
		event.reset(::WSACreateEvent());
		if (!event.get())
			throw_win32("WSACreateEvent");

		RIO_NOTIFICATION_COMPLETION note{};
		note.Type = RIO_EVENT_COMPLETION;
		note.Event.EventHandle = event.get();
		note.Event.NotifyReset = TRUE;

		// 队列深度 1024 条结果，足够容纳本示例的 8 个请求。
		// RIO_INVALID_CQ 表示失败，具体原因看 WSAGetLastError。
		const RIO_CQ cq = rio.RIOCreateCompletionQueue(1024, &note);
		if (cq == RIO_INVALID_CQ)
			throw_wsa("RIOCreateCompletionQueue");
		cq_guard.reset(rio, cq);
		std::printf("[3] 完成队列已创建（深度 1024，事件通知，自动复位）\n");

		// ---- 注册缓冲区（锁页） ----
		// 注册一次，后面所有 RIO_BUF 都只引用 BufferId + offset + length。
		// 缓冲区在 RIODeregisterBuffer 之前不能释放，也不能在注册范围内
		// 被普通 memcpy 之外的方式改掉（示例中就是普通内存）。
		const RIO_BUFFERID buffer_id =
			rio.RIORegisterBuffer(buffer.data(), static_cast<DWORD>(buffer.size()));
		if (buffer_id == RIO_INVALID_BUFFERID)
			throw_wsa("RIORegisterBuffer");
		buf_guard.reset(rio, buffer_id);
		std::printf("[4] 已注册 %zu 字节缓冲区，BufferId=%p\n", buffer.size(),
			reinterpret_cast<void *>(buffer_id));

		// ---- 为两个 socket 各建一个请求队列 ----
		// 参数：socket、最大未完成接收数、每次接收最多几个缓冲区、
		//       最大未完成发送数、每次发送最多几个缓冲区、
		//       接收完成队列、发送完成队列、SocketContext。
		// 这里收发共用一个 CQ；单次缓冲区数必须为 1（Windows 8/2012 的硬性要求）。
		// RQ 没有单独的 close 函数：socket 关闭时它自动释放，所以本示例把它
		// 的生命周期交给 unique_socket。
		const RIO_RQ recv_rq = rio.RIOCreateRequestQueue(recv_sock.get(),
			8, 1, 8, 1, cq, cq,
			reinterpret_cast<PVOID>(static_cast<std::uintptr_t>(0x1000)));
		if (recv_rq == RIO_INVALID_RQ)
			throw_wsa("RIOCreateRequestQueue(recv)");
		const RIO_RQ send_rq = rio.RIOCreateRequestQueue(send_sock.get(),
			8, 1, 8, 1, cq, cq,
			reinterpret_cast<PVOID>(static_cast<std::uintptr_t>(0x2000)));
		if (send_rq == RIO_INVALID_RQ)
			throw_wsa("RIOCreateRequestQueue(send)");
		std::printf("[5] 两个请求队列已创建\n");

		// ---- 先把 4 个接收请求挂上 ----
		// 每个请求一个 RIO_BUF：BufferId + 偏移 + 最大长度。请求完成时
		// CQ 里的 RIORESULT 会带回 RequestContext，用它在应用层找回请求。
		for (int i = 0; i < kMessages; ++i) {
			RIO_BUF rb{};
			rb.BufferId = buffer_id;
			rb.Offset = kRecvArea + static_cast<ULONG>(i) * kMaxDatagram;
			rb.Length = kMaxDatagram;
			const auto ctx = reinterpret_cast<PVOID>(
				static_cast<std::uintptr_t>(0x100 + i));
			if (!rio.RIOReceive(recv_rq, &rb, 1, 0, ctx))
				throw_wsa("RIOReceive");
		}
		std::printf("[6] 已挂 %d 个 RIOReceive\n", kMessages);

		// ---- 构造 4 个数据报并批量提交 ----
		// 每个 RIOSend 的 RIO_BUF 都指向互不重叠的区域：未完成的发送不能
		// 复用同一段发送缓冲区。
		std::array<std::string, kMessages> messages{};
		for (int i = 0; i < kMessages; ++i) {
			messages[i] = "RIO datagram #" + std::to_string(i);
			std::memcpy(buffer.data() + kSendArea +
					static_cast<std::size_t>(i) * kMaxDatagram,
				messages[i].data(), messages[i].size());

			RIO_BUF sb{};
			sb.BufferId = buffer_id;
			sb.Offset = kSendArea + static_cast<ULONG>(i) * kMaxDatagram;
			sb.Length = static_cast<ULONG>(messages[i].size());
			// RIO_MSG_DEFER：请求先入队但不立即执行，攒够一批再一起提交。
			// 它仍会占用 RQ 的“未完成发送”配额。
			const auto ctx = reinterpret_cast<PVOID>(
				static_cast<std::uintptr_t>(0x200 + i));
			if (!rio.RIOSend(send_rq, &sb, 1, RIO_MSG_DEFER, ctx))
				throw_wsa("RIOSend(DEFER)");
		}

		// 真正的“批量提交”：flags 只填 RIO_MSG_COMMIT_ONLY，
		// 数据指针、缓冲区数、RequestContext 都必须为空/0。
		// 这一下会把前面 4 个 DEFER 的发送一次性推给协议栈，
		// 比“最后一个不带 DEFER”的写法少一次慢路径完成。
		if (!rio.RIOSend(send_rq, nullptr, 0, RIO_MSG_COMMIT_ONLY, nullptr))
			throw_wsa("RIOSend(COMMIT_ONLY)");
		std::printf("[7] 4 个数据报已 DEFER 入队，并用 RIO_MSG_COMMIT_ONLY 批量提交\n");

		// ---- 收完成 ----
		// RIONotify 是“一次性门铃”：CQ 里出现新的完成结果时通知一次，
		// 之后要重新调用。上一次通知未完成时再调会得到 WSAEALREADY。
		std::array<RIORESULT, 16> results{};
		int completed = 0;
		int recv_seen = 0;
		while (completed < 2 * kMessages) {
			const int rc = rio.RIONotify(cq); // 成功返回 ERROR_SUCCESS(0)
			if (rc != ERROR_SUCCESS)
				throw std::runtime_error("RIONotify 返回错误码 " + std::to_string(rc));

			WSAEVENT h = event.get();
			const DWORD wait = ::WSAWaitForMultipleEvents(1, &h, TRUE, 5000, FALSE);
			if (wait == WSA_WAIT_TIMEOUT)
				throw std::runtime_error("等待 RIO 完成超时");
			if (wait != WSA_WAIT_EVENT_0)
				throw_win32("WSAWaitForMultipleEvents");

			// 一次最多取 results.size() 条；返回 RIO_CORRUPT_CQ 表示
			// 完成队列已损坏（通常是应用违反了“未完成请求不复用缓冲区”）。
			const ULONG n = rio.RIODequeueCompletion(cq, results.data(),
				static_cast<ULONG>(results.size()));
			if (n == RIO_CORRUPT_CQ)
				throw std::runtime_error("RIODequeueCompletion: 完成队列损坏");

			for (ULONG i = 0; i < n; ++i) {
				const RIORESULT &r = results[i];
				const auto ctx = static_cast<std::uintptr_t>(r.RequestContext);
				if (r.Status != 0) {
					std::printf("  完成 status=%ld ctx=0x%llx（失败）\n",
						static_cast<long>(r.Status),
						static_cast<unsigned long long>(ctx));
					throw std::runtime_error("RIO 请求失败，status=" +
						std::to_string(r.Status));
				}
				if (ctx >= 0x100 && ctx < 0x100 + kMessages) {
					const int idx = static_cast<int>(ctx - 0x100);
					const char *data = buffer.data() + kRecvArea +
						static_cast<std::size_t>(idx) * kMaxDatagram;
					std::printf("  [recv %d] %lu 字节: %.*s\n", idx,
						static_cast<unsigned long>(r.BytesTransferred),
						static_cast<int>(r.BytesTransferred), data);
					++recv_seen;
				} else {
					std::printf("  [send %llu] %lu 字节已发送\n",
						static_cast<unsigned long long>(ctx & 0xff),
						static_cast<unsigned long>(r.BytesTransferred));
				}
				++completed;
			}
		}

		std::printf("[8] 完成：%d 个请求完成（其中接收 %d 个），演示结束。\n",
			completed, recv_seen);
	} catch (const std::exception &e) {
		// 析构顺序：send/recv socket（释放 RQ）→ 注销缓冲区 → 关 CQ → 关事件。
		std::printf("失败：%s\n", e.what());
		return 1;
	}
	return 0;
}
