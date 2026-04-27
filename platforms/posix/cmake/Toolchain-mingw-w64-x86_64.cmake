############################################################################
#
# PX4 MinGW-w64 x86_64 cross toolchain.
#
# Used to cross-compile px4_sitl_default from a POSIX host to a native
# Windows x86_64 executable via the MinGW-w64 GCC port. Pick the
# *-posix* variant (winpthreads) so std::thread, pthread, and
# std::mutex work.
#
# Usage (from PX4-Autopilot repo root):
#   CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=Toolchain-mingw-w64-x86_64" \
#       make px4_sitl_default
#
# The bare toolchain name is resolved through CMAKE_MODULE_PATH, which
# cmake/kconfig.cmake extends with platforms/${PX4_PLATFORM}/cmake.
############################################################################

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(TOOLCHAIN_PREFIX x86_64-w64-mingw32)

find_program(MINGW_C_COMPILER NAMES ${TOOLCHAIN_PREFIX}-gcc-posix ${TOOLCHAIN_PREFIX}-gcc)
find_program(MINGW_CXX_COMPILER NAMES ${TOOLCHAIN_PREFIX}-g++-posix ${TOOLCHAIN_PREFIX}-g++)
find_program(MINGW_RC_COMPILER NAMES ${TOOLCHAIN_PREFIX}-windres)
find_program(MINGW_AR          NAMES ${TOOLCHAIN_PREFIX}-ar)
find_program(MINGW_RANLIB      NAMES ${TOOLCHAIN_PREFIX}-ranlib)

if(NOT MINGW_C_COMPILER OR NOT MINGW_CXX_COMPILER)
	message(FATAL_ERROR
		"MinGW-w64 (${TOOLCHAIN_PREFIX}) not found. "
		"Install with: apt-get install mingw-w64 g++-mingw-w64-x86-64-posix gcc-mingw-w64-x86-64-posix")
endif()

set(CMAKE_C_COMPILER   ${MINGW_C_COMPILER})
set(CMAKE_CXX_COMPILER ${MINGW_CXX_COMPILER})
set(CMAKE_RC_COMPILER  ${MINGW_RC_COMPILER})
set(CMAKE_AR           ${MINGW_AR})
set(CMAKE_RANLIB       ${MINGW_RANLIB})

# Find headers/libs only in the target sysroot, but let us run host tools
# (python, etc.) during the build (generators, kconfig, msg-gen).
# PACKAGE uses BOTH so find_package() can locate packages installed by
# nested ExternalProject builds (microcdr under the PX4 build tree) as
# well as packages living in the MinGW sysroot.
set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# MinGW lacks many POSIX headers. Put our shim directory in the search
# path ahead of the MinGW sysroot — do it here (rather than in the posix
# platform layer only) so that nested ExternalProject builds
# (Micro-XRCE-DDS-Client) using this toolchain also resolve <poll.h>,
# <sys/statfs.h>, <netdb.h>, <net/if.h>, <time.h> _r wrappers, etc.
get_filename_component(_px4_windows_shim_dir
	"${CMAKE_CURRENT_LIST_DIR}/../include/windows_shim" ABSOLUTE)
include_directories(BEFORE SYSTEM "${_px4_windows_shim_dir}")

# Target Windows 10 (1803+) so AF_UNIX is available in WinSock2.
add_compile_definitions(
	_WIN32_WINNT=0x0A00       # Windows 10
	WINVER=0x0A00
	__PX4_WINDOWS             # PX4 platform selector
	NOMINMAX                  # avoid <windows.h> min/max macros
	WIN32_LEAN_AND_MEAN
	_USE_MATH_DEFINES
	__USE_MINGW_ANSI_STDIO=1  # make printf accept PRIu64 / %llu / %zu
)

# PX4 uses `#pragma pack(push, 1)` around structs with multi-bit bitfields
# (e.g. sixteen 11-bit RC channel fields in src/lib/rc/crsf.cpp). MinGW
# defaults to the MSVC bitfield ABI, which lays those out differently
# from Linux GCC (each storage unit padded to the declared type's
# alignment instead of packed bit-adjacent). Force the Itanium/SysV
# layout so decoded wire formats match across platforms.
add_compile_options(-mno-ms-bitfields)

# Produce a statically linked .exe so the user does not need to ship
# libgcc_s, libstdc++, and winpthread DLLs separately.
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-static -static-libgcc -static-libstdc++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static-libgcc -static-libstdc++")

# ctest runs the PE binary via wine through the CROSSCOMPILING_EMULATOR
# property set on the px4 target (platforms/posix/CMakeLists.txt),
# rather than CMAKE_CROSSCOMPILING_EMULATOR here, because the latter
# does not reliably propagate across CMake reconfigures.
