cmake_minimum_required(VERSION 2.8.4)

set(CMAKE_SUPPRESS_REGENERATION TRUE)
add_definitions(-DPL150)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin5)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

option(BUILD_TESTS "Build tests." OFF)
option(BUILD_BENCHMARKS "Build benchmarks." OFF)
option(PYTHON "Build for Python library." OFF)
option(BUILD_GRAPHICS "Build in the graphics library." OFF)
#option(INTERNAL "Build internal portions of the library." OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(top_sources
    include/vn/exceptions.h
    include/vn/int.h
    include/vn/utilities.h
    include/vn/event.h
    src/vn/utilities.cpp
    src/vn/event.cpp)
source_group("" FILES ${top_sources})

set(math_sources
    include/vn/math/consts.h
    include/vn/math/math.h
    include/vn/math/conversions.h
    include/vn/math/vector.h
    include/vn/math/matrix.h
    include/vn/math/kinematics.h
    include/vn/math/attitude.h
    include/vn/math/position.h
    src/vn/math/conversions.cpp
    src/vn/math/attitude.cpp
    src/vn/math/position.cpp)
source_group("math" FILES ${math_sources})

set(util_sources
    include/vn/util/export.h
    include/vn/util/boostpython.h
    include/vn/util/compiler.h
    include/vn/util/memoryport.h
	include/vn/util/nocopy.h
    src/vn/util/memoryport.cpp)
#if (PYTHON)
#    set(util_sources
#        ${util_sources}
#        include/vn/util/dllvalidator.h
#        src/vn/util/dllvalidator.cpp)
#endif()
source_group("util" FILES ${util_sources})

set(data_sources
    include/vn/data/error_detection.h
    src/vn/data/error_detection.cpp)
source_group("data" FILES ${data_sources})

set(protocol_uart_sources
    include/vn/protocol/uart/types.h
	include/vn/protocol/uart/util.h
	include/vn/protocol/uart/packet.h
	include/vn/protocol/uart/packetfinder.h
	src/vn/protocol/uart/types.cpp
	src/vn/protocol/uart/util.cpp
	src/vn/protocol/uart/packet.cpp
	src/vn/protocol/uart/packetfinder.cpp)
source_group("protocol\\uart" FILES ${protocol_uart_sources})

set(sensors_sources
	include/vn/sensors/registers.h
	include/vn/sensors/mock.h
    include/vn/sensors/sensors.h
	include/vn/sensors/searcher.h
	include/vn/sensors/compositedata.h
	include/vn/sensors/ezasyncdata.h
	src/vn/sensors/sensors.cpp
	src/vn/sensors/searcher.cpp
	src/vn/sensors/compositedata.cpp
	src/vn/sensors/ezasyncdata.cpp)
source_group("sensors" FILES ${sensors_sources})

set(xplat_sources
    include/vn/xplat/criticalsection.h
	include/vn/xplat/event.h
	include/vn/xplat/serialport.h
	include/vn/xplat/signal.h
	include/vn/xplat/thread.h
	include/vn/xplat/time.h
	include/vn/xplat/port.h
	src/vn/xplat/criticalsection.cpp
	src/vn/xplat/event.cpp
	src/vn/xplat/serialport.cpp
	src/vn/xplat/thread.cpp
	src/vn/xplat/time.cpp
	src/vn/xplat/port.cpp)
source_group("xplat" FILES ${xplat_sources})

set(graphics_sources
	include/vn/graphics/color.h
	include/vn/graphics/sphere.h
	include/vn/graphics/geometry.h
	include/vn/graphics/multicolor.h
	include/vn/graphics/lines.h
	include/vn/graphics/math.h
	include/vn/graphics/frame.h
	src/vn/graphics/color.cpp
	src/vn/graphics/sphere.cpp
	src/vn/graphics/geometry.cpp
	src/vn/graphics/multicolor.cpp
	src/vn/graphics/lines.cpp
	src/vn/graphics/math.cpp
	src/vn/graphics/util.cpp
	src/vn/graphics/frame.cpp)
source_group("" FILES ${graphics_sources})

set(graphics_math_sources
	include/vn/graphics/math/vector.h
	include/vn/graphics/math/matrix.h)
source_group("math" FILES ${graphics_math_sources})


include_directories(include)

if (UNIX)
	include_directories(
		/user/include/python3.4)
endif()

#if (PYTHON)
#	# Python needs this as a DLL to load into the Python interpretor.
#	add_library(proglib-cpp SHARED
#	    ${top_sources}
#	    ${math_sources}
#	    ${util_sources}
#	    ${data_sources}
#	    ${protocol_uart_sources}
#	    ${sensors_sources}
#	    ${xplat_sources})
#
#	if(WIN32)
#		target_compile_definitions(proglib-cpp PUBLIC -D_BUILD_DLL)
#		target_compile_definitions(proglib-cpp PUBLIC -Dproglib_cpp_EXPORTS)
#
#		set_target_properties(
#			proglib-cpp PROPERTIES
#			RUNTIME_OUTPUT_DIRECTORY "../../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_RELEASE "../../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_DEBUG "../../vnpy"
#			LIBRARY_OUTPUT_DIRECTORY "../../vnpy")
#
#		include_directories(../libs/PeLib/include)
#		target_link_libraries(proglib-cpp LINK_PUBLIC PeLib)
#		add_subdirectory(../libs/PeLib PeLib)
#
#		set_target_properties(
#			PeLib PROPERTIES
#			RUNTIME_OUTPUT_DIRECTORY "../../../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_RELEASE "../../../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_DEBUG "../../../vnpy"
#			LIBRARY_OUTPUT_DIRECTORY "../../../vnpy")
#	elseif(UNIX)
#		set_target_properties(
#			proglib-cpp PROPERTIES
#			RUNTIME_OUTPUT_DIRECTORY "../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_RELEASE "../vnpy"
#			RUNTIME_OUTPUT_DIRECTORY_DEBUG "../vnpy"
#			LIBRARY_OUTPUT_DIRECTORY "../vnpy")
#	endif()
#else()
add_library(vnproglib
    ${top_sources}
    ${math_sources}
    ${util_sources}
    ${data_sources}
    ${protocol_uart_sources}
    ${sensors_sources}
    ${xplat_sources})
#
#	if(WIN32)
#		target_link_libraries(proglib-cpp PRIVATE Setupapi)
#	endif()
#endif()
#
#if (BUILD_GRAPHICS)
#
#	include_directories(
#		../libs/glew/include
#		../libs/glm)
#
#
#	#target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
##	set(CMAKE_PREFIX_PATH "../libs/glew")
##	set(CMAKE_LIBRARY_PATH "../libs/glew/lib/Release/Win32/")
#
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
##	find_package(GLEW REQUIRED)
#	#add_library(GLEW_shared INTERFACE IMPORTED)
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
#
#	#message("${GLEW_FOUND}")
#
##	include_directories(${GLEW_INCLUDE_DIRS})
##    link_libraries(${GLEW_LIBRARIES})
#
#
##    add_subdirectory(../libs/glew GLEW_shared)
#
#
#	add_library(proglib-cpp-graphics SHARED
#		${graphics_sources}
#		${graphics_math_sources})
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -D_BUILD_DLL)
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -Dproglib_cpp_graphics_EXPORTS)
#	target_link_libraries(proglib-cpp-graphics PRIVATE OpenGL32)
#	target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
#endif()
#
#if (WIN32)
#	target_link_libraries(proglib-cpp PRIVATE Setupapi)

#	if (PYTHON)
#		include_directories(../libs/PeLib/include)
#		target_link_libraries(proglib-cpp LINK_PUBLIC PeLib)
#		add_subdirectory(../libs/PeLib PeLib)
#	endif()
#endif()


# The properties below are mainly for the Python library.
#if (WIN32)
#set_target_properties(
#	proglib-cpp PROPERTIES
#	RUNTIME_OUTPUT_DIRECTORY "../../../vnpy"
#	RUNTIME_OUTPUT_DIRECTORY_RELEASE "../../../vnpy"
#	RUNTIME_OUTPUT_DIRECTORY_DEBUG "../../../vnpy"
#	LIBRARY_OUTPUT_DIRECTORY "../../../vnpy")
#elseif(UNIX)
#set_target_properties(
#	proglib-cpp PROPERTIES
#	RUNTIME_OUTPUT_DIRECTORY "../vnpy"
#	RUNTIME_OUTPUT_DIRECTORY_RELEASE "../vnpy"
#	RUNTIME_OUTPUT_DIRECTORY_DEBUG "../vnpy"
#	LIBRARY_OUTPUT_DIRECTORY "../vnpy")
#endif()

#if (BUILD_TESTS)
#
#    file(GLOB_RECURSE TEST_SOURCE_FILES src/vn/**/*.test.cpp)
#    set(TEST_SOURCE_FILES ${TEST_SOURCE_FILES} src/vn/main.cpp)
#
#	set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries (DLLs).")
#	add_subdirectory(../libs/googletest gtest)
#
#	include_directories(../libs/googletest/include)
#
#	add_executable(proglib-cpp-test ${TEST_SOURCE_FILES})
#
#	target_link_libraries(proglib-cpp-test LINK_PUBLIC gtest)
#	target_link_libraries(proglib-cpp-test LINK_PUBLIC proglib-cpp)
#
#endif()
#
#if (BUILD_BENCHMARKS)
#
#    file(GLOB_RECURSE BENCHMARK_SOURCE_FILES src/vn/**/*.benchmark.cpp)
#    set(BENCHMARK_SOURCE_FILES ${BENCHMARK_SOURCE_FILES} src/vn/benchmark.cpp)
#
#	add_subdirectory(../libs/hayai hayai)
#	add_subdirectory(../libs/glew GLEW_shared)
#
#	include_directories(../libs/hayai/src)
#
#	add_executable(proglib-cpp-benchmark ${BENCHMARK_SOURCE_FILES})
#
#	#target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC hayai)
#	target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC proglib-cpp)
#	target_link_libraries(proglib-cpp-benchmark LINK_PUBLIC proglib-cpp-graphics)
#	target_link_libraries(proglib-cpp-graphics PRIVATE OpenGL32)
#	target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
#endif()
