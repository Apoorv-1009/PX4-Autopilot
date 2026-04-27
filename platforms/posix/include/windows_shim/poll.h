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
 * @file poll.h
 *
 * Maps POSIX poll() onto Win32 WSAPoll(). WSAPoll has the same struct
 * layout and event flags as POSIX poll; it has been available since
 * Vista but only works on sockets, which is exactly how PX4 uses it
 * (px4_daemon socket pair, muorb sockets, mavlink).
 */
#pragma once

#include <winsock2.h>

#ifdef __cplusplus
extern "C" {
#endif

/* winsock2 already defines pollfd / WSAPOLLFD and the event bits when
 * _WIN32_WINNT >= 0x0600 (Vista). Re-export as the POSIX-style names. */

#ifndef POLLRDNORM
#define POLLRDNORM POLLIN
#endif
#ifndef POLLRDBAND
#define POLLRDBAND 0x0080
#endif
#ifndef POLLWRNORM
#define POLLWRNORM POLLOUT
#endif
#ifndef POLLWRBAND
#define POLLWRBAND 0x0100
#endif
#ifndef POLLMSG
#define POLLMSG 0x0400
#endif
#ifndef POLLREMOVE
#define POLLREMOVE 0x1000
#endif

typedef struct pollfd POLLFD;   /* alias for readability */

typedef unsigned long nfds_t;

/**
 * @brief Poll socket descriptors using WSAPoll().
 *
 * @return Number of ready descriptors, 0 on timeout, or SOCKET_ERROR.
 */
static inline int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	return WSAPoll(fds, (ULONG)nfds, timeout);
}

#ifdef __cplusplus
}
#endif
