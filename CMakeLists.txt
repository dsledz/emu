cmake_minimum_required(VERSION 2.6)
project(SLEDEMU)

include_directories("/opt/local/include")

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(THREADS_PREFER_PTHREAD_FLAG ON)

IF (WIN32)

ELSE()
    set(CMAKE_CXX_FLAGS "-Wall -Werror -g -std=c++11 -stdlib=libc++ -Wno-unused-private-field -Wuninitialized -O0")
    set(CMAKE_CC "clang++")
ENDIF()

# XCode options
set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvm.clang.1_0")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
set(CMAKE_XCODE_EFFECTIVE_PLATFORMS "-iphoneos;-iphonesimulator")

set(MACOSX_BUNDLE_GUI_IDENTIFIER "org.sledz.\${PRODUCT_NAME:identifier}")
set(APP_TYPE MACOSX_BUNDLE)

macro(add_framework fwname)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework ${fwname}")
endmacro(add_framework)

option(BUILD_ASM "Build ASM" ON)
option(BUILD_IOS "Build iOS Support" OFF)
option(BUILD_OSX "Build OSX" OFF)
option(BUILD_SDL "Build SDL" OFF)
option(BUILD_SFML "Build SFML" OFF)
option(BUILD_TESTING "Enable testing" ON)
option(BUILD_CORE_ONLY "Enable core only" OFF)
option(BUILD_JIT "Build JIT" OFF)
option(BUILD_LUA "Build LUA" ON)
option(BUILD_NES "Build NES" ON)
option(BUILD_TG16 "Build TG-16" ON)
option(BUILD_GAMEBOY "Build GameBoy" ON)
option(BUILD_C64 "Build C64" ON)
option(BUILD_ARCADE "Build Arcade" ON)
option(BUILD_SBC "Build SBC" ON)

set(TARGET_ARCH "x64" CACHE STRING "Target Architecture")

set(OPENGL_LEGACY 1)
if (BUILD_IOS)
    set(BUILD_TESTING 0)
endif (BUILD_IOS)

if (BUILD_JIT)
    set(M6502_JIT 1)
endif (BUILD_JIT)

# Gtest
if (BUILD_TESTING)
    message("Building Testing")
    set(GTEST_DIR ${PROJECT_SOURCE_DIR}/gtest-1.7.0)
    set(GTEST_USE_OWN_TR1_TUPLE 1)
    set(GTEST_HAS_TR1_TUPLE 0)
    add_subdirectory(${GTEST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/gtest)
    include_directories(${GTEST_DIR}/include)
    Include(CTest)
    enable_testing(true)
endif (BUILD_TESTING)

if (BUILD_CORE_ONLY)
    add_subdirectory(core)
    add_subdirectory(lua)
else (!BUILD_CORE_ONLY)
    add_subdirectory(core)
    add_subdirectory(lua)
    add_subdirectory(emu)
    add_subdirectory(cpu)
    add_subdirectory(machine)
    if (BUILD_IOS OR BUILD_OSX OR BUILD_SDL OR BUILD_SFML)
        add_subdirectory(driver)
    endif()
endif (BUILD_CORE_ONLY)

configure_file(
    "${PROJECT_SOURCE_DIR}/opts.h.in"
    "${PROJECT_BINARY_DIR}/opts.h")

