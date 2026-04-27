/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file init.cpp
 *
 * One-time process init for the Windows POSIX-shim subsystem.
 *
 * Owns WSAStartup/WSACleanup, binary-mode stdio, Console Output
 * Code Page selection, native Windows ANSI virtual-terminal processing,
 * and the Wine /dev/tty termios restore path invoked by Ctrl+C handling
 * in main.cpp.
 */

#include "px4_windows_internal.h"

#include <array>

/* --------------------------------------------------------------------------
 * One-time process-wide initialisation.
 *
 * WinSock requires WSAStartup before any socket call, and we want a
 * UTF-8 console so PX4_INFO banner output isn't mangled. Install a
 * global constructor that runs before main().
 * -------------------------------------------------------------------------- */
/* Session id claimed by the first setsid() caller - shared process-wide
 * via InterlockedCompareExchange from proc/ids.cpp. External linkage so
 * the extern declaration in proc/ids.cpp resolves. */
volatile LONG g_px4_session_id = 0;

namespace
{

bool px4_is_running_under_wine()
{
	if (HMODULE ntdll = GetModuleHandleA("ntdll.dll")) {
		return GetProcAddress(ntdll, "wine_get_version") != nullptr;
	}

	return false;
}

struct PX4WindowsGlobalInit {
	struct SavedConsoleMode {
		HANDLE handle = nullptr;
		DWORD  mode   = 0;
		bool   valid  = false;
	};

	struct LinuxTermios {
		unsigned int c_iflag;
		unsigned int c_oflag;
		unsigned int c_cflag;
		unsigned int c_lflag;
		unsigned char c_line;
		unsigned char c_cc[19];
	};

	SavedConsoleMode saved_stdin;
	SavedConsoleMode saved_stdout;
	SavedConsoleMode saved_stderr;
	LinuxTermios saved_host_tty {};
	bool saved_host_tty_valid = false;
	bool saved_host_tty_was_cooked = false;
	const bool running_under_wine = px4_is_running_under_wine();

	static constexpr long long LX_SYS_open   = 2;
	static constexpr long long LX_SYS_close  = 3;
	static constexpr long long LX_SYS_ioctl  = 16;
	static constexpr long long LX_O_RDWR     = 2;
	static constexpr long long LX_O_NOCTTY   = 0x100;
	static constexpr long long LX_TCGETS     = 0x5401;
	static constexpr long long LX_TCSETS     = 0x5402;
	// Input flags
	static constexpr unsigned int LX_BRKINT = 0x0002, LX_IGNPAR = 0x0004,
				      LX_ICRNL  = 0x0100, LX_IXON   = 0x0400;
	// Output flags
	static constexpr unsigned int LX_OPOST  = 0x0001, LX_ONLCR  = 0x0004;
	// Control flags
	static constexpr unsigned int LX_CSIZE  = 0x0030, LX_CS8    = 0x0030,
				      LX_CREAD  = 0x0080;
	// Local flags
	static constexpr unsigned int LX_ISIG    = 0x0001, LX_ICANON  = 0x0002,
				      LX_ECHO    = 0x0008, LX_ECHOE   = 0x0010,
				      LX_ECHOK   = 0x0020, LX_ECHOCTL = 0x0200,
				      LX_ECHOKE  = 0x0800, LX_IEXTEN  = 0x8000;

	DWORD cooked_stdin_mode() const
	{
		DWORD mode = ENABLE_PROCESSED_INPUT | ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT;

#ifdef ENABLE_EXTENDED_FLAGS
		mode |= ENABLE_EXTENDED_FLAGS;
#endif
#ifdef ENABLE_INSERT_MODE
		mode |= ENABLE_INSERT_MODE;
#endif
#ifdef ENABLE_QUICK_EDIT_MODE
		mode |= ENABLE_QUICK_EDIT_MODE;
#endif

		return mode;
	}

	// Inline Linux syscall helpers (x86_64 ABI).
	static long long linux_syscall1(long long num, long long a)
	{
		long long ret;
		__asm__ volatile (
			"syscall"
			: "=a"(ret)
			: "0"(num), "D"(a)
			: "rcx", "r11", "memory"
		);
		return ret;
	}

	static long long linux_syscall3(long long num, long long a, long long b, long long c)
	{
		long long ret;
		__asm__ volatile (
			"syscall"
			: "=a"(ret)
			: "0"(num), "D"(a), "S"(b), "d"(c)
			: "rcx", "r11", "memory"
		);
		return ret;
	}

	static long long open_host_tty()
	{
		return linux_syscall3(LX_SYS_open,
				      reinterpret_cast<long long>("/dev/tty"),
				      LX_O_RDWR | LX_O_NOCTTY, 0);
	}

	static bool tcget_host_tty(LinuxTermios &term)
	{
		long long fd = open_host_tty();

		if (fd < 0) {
			return false;
		}

		const long long ret = linux_syscall3(LX_SYS_ioctl, fd, LX_TCGETS, reinterpret_cast<long long>(&term));
		(void)linux_syscall1(LX_SYS_close, fd);
		return ret == 0;
	}

	static bool tcset_host_tty(const LinuxTermios &term)
	{
		long long fd = open_host_tty();

		if (fd < 0) {
			return false;
		}

		const long long ret = linux_syscall3(LX_SYS_ioctl, fd, LX_TCSETS, reinterpret_cast<long long>(&term));
		(void)linux_syscall1(LX_SYS_close, fd);
		return ret == 0;
	}

	static bool termios_is_interactive_cooked(const LinuxTermios &term)
	{
		const unsigned int required_lflag = LX_ISIG | LX_ICANON | LX_ECHO;
		return (term.c_lflag & required_lflag) == required_lflag;
	}

	static void make_cooked_termios(LinuxTermios &term)
	{
		term.c_iflag = LX_BRKINT | LX_IGNPAR | LX_ICRNL | LX_IXON;
		term.c_oflag = LX_OPOST | LX_ONLCR;
		term.c_cflag = (term.c_cflag & ~LX_CSIZE) | LX_CS8 | LX_CREAD;
		term.c_lflag = LX_ISIG | LX_ICANON | LX_ECHO | LX_ECHOE | LX_ECHOK
				| LX_ECHOCTL | LX_ECHOKE | LX_IEXTEN;

		term.c_cc[0]  = 3;    // VINTR    Ctrl+C
		term.c_cc[1]  = 28;   // VQUIT    Ctrl-backslash
		term.c_cc[2]  = 127;  // VERASE   DEL
		term.c_cc[3]  = 21;   // VKILL    Ctrl+U
		term.c_cc[4]  = 4;    // VEOF     Ctrl+D
		term.c_cc[5]  = 0;    // VTIME
		term.c_cc[6]  = 1;    // VMIN
		term.c_cc[8]  = 17;   // VSTART   Ctrl+Q
		term.c_cc[9]  = 19;   // VSTOP    Ctrl+S
		term.c_cc[10] = 26;   // VSUSP    Ctrl+Z
		term.c_cc[12] = 18;   // VREPRINT Ctrl+R
		term.c_cc[13] = 15;   // VDISCARD Ctrl+O
		term.c_cc[14] = 23;   // VWERASE  Ctrl+W
		term.c_cc[15] = 22;   // VLNEXT   Ctrl+V
		term.c_cc[16] = 0;    // VEOL2
	}

	// Restore the host Linux tty when PX4 exits under Wine. Wine 6.x does
	// not reliably translate our SetConsoleMode writes back into tcsetattr()
	// on the launching terminal, which can leave bash without working
	// readline/history keys after Ctrl+C.
	//
	// Prefer the exact /dev/tty termios snapshot captured at process init.
	// That preserves user-specific tty defaults instead of approximating
	// them with `stty sane`. If Wine had already put the tty in raw mode
	// before our constructor ran, fall back to a conservative cooked mode.
	//
	// MinGW ships no <termios.h>, so the x86_64 Linux syscall numbers and
	// struct layout are spelled out above.
	void restore_host_tty_via_wine_unix()
	{
		if (saved_host_tty_valid && saved_host_tty_was_cooked) {
			(void)tcset_host_tty(saved_host_tty);
			return;
		}

		LinuxTermios term {};

		if (!tcget_host_tty(term)) {
			return;
		}

		make_cooked_termios(term);
		(void)tcset_host_tty(term);
	}

	void restore_console_modes()
	{
		if (saved_stdin.valid)  { SetConsoleMode(saved_stdin.handle,  running_under_wine ? cooked_stdin_mode() : saved_stdin.mode); }
		if (saved_stdout.valid) { SetConsoleMode(saved_stdout.handle, saved_stdout.mode); }
		if (saved_stderr.valid) { SetConsoleMode(saved_stderr.handle, saved_stderr.mode); }

		if (running_under_wine) {
			restore_host_tty_via_wine_unix();
		}
	}

	PX4WindowsGlobalInit()
	{
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			fprintf(stderr, "PX4: WSAStartup failed\n");
		}
		SetConsoleOutputCP(CP_UTF8);

		// PX4 stores binary data (parameters.bson, dataman) and expects
		// read/write to preserve bytes exactly. MSVCRT's default text
		// mode maps CRLF<->LF, which corrupts arbitrary binary content.
		// MSVCRT exposes the global default mode through the
		// `__p__fmode()` accessor; setting it is equivalent to linking
		// against binmode.o or compiling the whole image with -D_O_BINARY.
		if (int *fmode_ptr = __p__fmode()) { *fmode_ptr = _O_BINARY; }
		_setmode(_fileno(stdin),  _O_BINARY);
		_setmode(_fileno(stdout), _O_BINARY);
		_setmode(_fileno(stderr), _O_BINARY);

		if (running_under_wine) {
			saved_host_tty_valid = tcget_host_tty(saved_host_tty);
			saved_host_tty_was_cooked = saved_host_tty_valid
						     && termios_is_interactive_cooked(saved_host_tty);
		}

		// Snapshot console modes so we can restore them on exit. Under
		// Wine, enabling VT on the output handles also flips the host
		// Linux tty into raw mode; if we don't write the original mode
		// back, the shell that launched wine is left with broken
		// arrows / Ctrl+C. On real Windows the restore is a no-op but
		// keeps us honest.
		auto snapshot = [](DWORD which, SavedConsoleMode &slot) {
			HANDLE h = GetStdHandle(which);
			if (h == INVALID_HANDLE_VALUE || h == nullptr) { return; }
			DWORD mode = 0;
			if (!GetConsoleMode(h, &mode)) { return; }
			slot.handle = h;
			slot.mode   = mode;
			slot.valid  = true;
		};
		snapshot(STD_INPUT_HANDLE,  saved_stdin);
		snapshot(STD_OUTPUT_HANDLE, saved_stdout);
		snapshot(STD_ERROR_HANDLE,  saved_stderr);

		// Opt native Windows consoles into ANSI escape processing. Under Wine,
		// px4_log writes ANSI-colored buffers directly to the host stdout fd
		// so Wine's console renderer cannot split escape sequences.
		// DISABLE_NEWLINE_AUTO_RETURN prevents the console from inserting an
		// extra CR at column 80, which otherwise shows up as spurious line wraps.
		const DWORD vt_flags = ENABLE_VIRTUAL_TERMINAL_PROCESSING
				     | DISABLE_NEWLINE_AUTO_RETURN;

		if (!running_under_wine) {
			if (saved_stdout.valid) {
				SetConsoleMode(saved_stdout.handle, saved_stdout.mode | vt_flags);
			}

			if (saved_stderr.valid) {
				SetConsoleMode(saved_stderr.handle, saved_stderr.mode | vt_flags);
			}
		}
	}
	~PX4WindowsGlobalInit()
	{
		restore_console_modes();
		WSACleanup();
	}
};
static PX4WindowsGlobalInit _px4_win_init;
} // namespace

extern "C" void px4_windows_restore_console_modes()
{
	_px4_win_init.restore_console_modes();
}

extern "C" void px4_windows_discard_pending_input()
{
	HANDLE stdin_h = GetStdHandle(STD_INPUT_HANDLE);

	if (stdin_h == INVALID_HANDLE_VALUE || stdin_h == nullptr) {
		return;
	}

	CancelIoEx(stdin_h, nullptr);

	DWORD mode = 0;

	if (GetConsoleMode(stdin_h, &mode)) {
		FlushConsoleInputBuffer(stdin_h);
		return;
	}

	if (GetFileType(stdin_h) == FILE_TYPE_PIPE) {
		char buffer[256];
		DWORD available = 0;

		while (PeekNamedPipe(stdin_h, nullptr, 0, nullptr, &available, nullptr) && available > 0) {
			DWORD read_count = 0;
			const DWORD to_read = (available < sizeof(buffer)) ? available : sizeof(buffer);

			if (!ReadFile(stdin_h, buffer, to_read, &read_count, nullptr) || read_count == 0) {
				break;
			}
		}
	}
}

extern "C" void px4_windows_release_console()
{
	_px4_win_init.restore_console_modes();

	if (!_px4_win_init.running_under_wine) {
		FreeConsole();
	}
}

extern "C" void px4_windows_exit(int status)
{
	fflush(stdout);
	fflush(stderr);
	_px4_win_init.restore_console_modes();

	if (!_px4_win_init.running_under_wine) {
		FreeConsole();
	}

	if (_px4_win_init.running_under_wine) {
		TerminateProcess(GetCurrentProcess(), static_cast<UINT>(status));
	}

	ExitProcess(static_cast<UINT>(status));
}
