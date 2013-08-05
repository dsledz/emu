cmake_minimum_required(VERSION 2.6)
project(SLEDEMU)

include_directories(${SLEDEMU_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "-Wall -Werror -g -std=c++11 -stdlib=libc++")
set(CMAKE_CC "clang++")

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

option(BUILD_IOS "Build iOS Support" OFF)
option(BUILD_OSX "Build OSX" ON)
option(BUILD_SDL "Build SDL" ON)
option(BUILD_TESTING "Enable testing" ON)

if (BUILD_IOS)
    set(BUILD_TESTING 0)
endif (BUILD_IOS)

# Gtest
if (BUILD_TESTING)
    message("Building Testing")
    set(GTEST_DIR ${SLEDEMU_SOURCE_DIR}/../gtest-1.6.0)
    set(GTEST_USE_OWN_TR1_TUPLE 1)
    set(GTEST_HAS_TR1_TUPLE 0)
    add_subdirectory(${GTEST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/gtest)
    include_directories(${GTEST_DIR}/include)
    Include(CTest)
    enable_testing(true)
endif (BUILD_TESTING)

add_subdirectory(emu)
add_subdirectory(cpu)
add_subdirectory(machine)
add_subdirectory(driver)

