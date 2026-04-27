/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file platform.h
 *
 * Public Windows-host hooks for the POSIX platform implementation.
 */

#pragma once

#include <visibility.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Restore the console modes captured during Windows platform startup.
 *
 * PX4 enables virtual-terminal output and may switch the input console into a
 * raw-ish mode for the pxh shell. This must run before process teardown so a
 * native console, a Wine-hosted terminal, or a parent shell does not inherit
 * stale input flags.
 */
void px4_windows_restore_console_modes(void);

/**
 * @brief Drop console input that was queued while PX4 was shutting down.
 *
 * Wine can leave bytes for control sequences or buffered pxh input pending in
 * the Linux terminal after Ctrl+C. The shutdown path calls this before
 * returning control to the parent shell.
 */
void px4_windows_discard_pending_input(void);

/**
 * @brief Release Windows console resources owned by the PX4 process.
 *
 * Native Windows builds use this as the last console cleanup step; under Wine
 * it complements the terminal-mode restore and input discard hooks above.
 */
void px4_windows_release_console(void);

/**
 * @brief Exit PX4 after running Windows-specific console cleanup.
 *
 * @param status Process exit code passed to the C runtime after cleanup.
 */
void px4_windows_exit(int status) noreturn_function;

#ifdef __cplusplus
}
#endif
