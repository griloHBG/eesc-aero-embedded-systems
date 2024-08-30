cmake_minimum_required(VERSION 3.18)
if (ARM_TARGET)
    message(STATUS "ARM INDEED!")

    set(CMAKE_SYSTEM_PROCESSOR arm)

    set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
else()
    message(STATUS "X86_64 INSTEAD OF ARM!")
endif()

set(CMAKE_CXX_STANDARD 17) # limited to 17 because of the toolchain T.T
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

project(eesc-aero-embedded-systems)

find_package(PkgConfig REQUIRED)
#find_package(Threads REQUIRED)

pkg_check_modules(LELY-COAPP REQUIRED IMPORTED_TARGET liblely-coapp)
#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

#set(CMAKE_EXE_LINKER_FLAGS "-static")

#set(CMAKE_CXX_FLAGS "--coverage")

#target_link_libraries(${PROJECT_NAME} Threads::Threads)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_executable(${PROJECT_NAME} main.cpp ManoplaLelyBBB.cpp utils/utils.cpp utils/json.h)

#target_include_directories(${PROJECT_NAME} PRIVATE PkgConfig::LELY-IO2)

#target_link_directories(${PROJECT_NAME} PRIVATE /home/grilo/Git/lely-core/install-arm-gcc-6.3/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LELY-COAPP_LIBRARIES} blas armadillo)
target_link_directories(${PROJECT_NAME} PRIVATE ${LELY-COAPP_LIBDIR})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic -g3)
if (ARM_TARGET)
    target_include_directories(${PROJECT_NAME} PRIVATE ${LELY-COAPP_INCLUDEDIR})
    #target_include_directories(${PROJECT_NAME} PRIVATE ${BBB_DEBIAN_11_SYSROOT}/usr/include)
endif ()
target_include_directories(${PROJECT_NAME} PRIVATE utils)#target_include_directories(${PROJECT_NAME} PRIVATE ./gnuplot-iostream)
#install(CODE "execute_process(COMMAND scp ${CMAKE_BINARY_DIR}/${PROJECT_NAME} debian@192.168.7.2:/home/debian/${PROJECT_NAME})")