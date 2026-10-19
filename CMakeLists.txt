cmake_minimum_required(VERSION 3.20)
project(nvgt_steamaudio LANGUAGES CXX)

# Same toolchain settings as NVGT itself: C++20 and a static C runtime (/MT), so the DLL loads on a machine
# without the Visual C++ Redistributable.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Steam Audio SDK (https://valvesoftware.github.io/steam-audio/), the same 4.x release NVGT's vcpkg port uses.
# The directory must contain include/phonon.h, lib/phonon.lib and bin/phonon.dll.
set(STEAMAUDIO_DIR "${CMAKE_SOURCE_DIR}/../nvgt_steamaudio_occlusion/third_party/steamaudio" CACHE PATH "Steam Audio SDK directory")

add_library(nvgt_steamaudio SHARED src/nvgt_steamaudio.cpp)
# third_party/nvgt/nvgt_plugin.h is a copy of nvgt/src/nvgt_plugin.h, third_party/angelscript/angelscript.h the
# header of the AngelScript version NVGT is built with. Only the headers are needed: the plugin receives the
# AngelScript functions from NVGT at load time.
target_include_directories(nvgt_steamaudio PRIVATE
	${CMAKE_SOURCE_DIR}/third_party/nvgt
	${CMAKE_SOURCE_DIR}/third_party/angelscript
	${STEAMAUDIO_DIR}/include
)
target_link_libraries(nvgt_steamaudio PRIVATE ${STEAMAUDIO_DIR}/lib/phonon.lib)
if (MSVC)
	target_compile_options(nvgt_steamaudio PRIVATE /EHsc /utf-8 /permissive- /W3)
endif()
set_target_properties(nvgt_steamaudio PROPERTIES PREFIX "")
