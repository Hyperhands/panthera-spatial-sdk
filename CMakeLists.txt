cmake_minimum_required(VERSION 3.22)
project(panthera_spatial_sdk LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
include(CTest)

add_subdirectory(third_party/wirelink EXCLUDE_FROM_ALL)
set(BLASFEO_TARGET GENERIC CACHE STRING "BLASFEO instruction set for SDK binaries")
# Keep acados from selecting its source checkout as the installation prefix.
set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
add_subdirectory(third_party/acados EXCLUDE_FROM_ALL)

add_library(panthera_eigen INTERFACE)
target_include_directories(panthera_eigen SYSTEM INTERFACE third_party/eigen)
set(PYBIND11_FINDPYTHON ON)
add_subdirectory(third_party/pybind11 EXCLUDE_FROM_ALL)

add_library(panthera_protocol STATIC
    generated/protocol/robot_messages.c
    generated/protocol/robot_messages_bindings.c
    generated/protocol/robot_messages_runtime.c)
target_include_directories(panthera_protocol PUBLIC generated/protocol)
target_link_libraries(panthera_protocol PUBLIC Wirelink::wirelink)

# Build the QP backend even when the integration test is disabled.
add_custom_target(panthera_host ALL DEPENDS panthera_protocol acados)

if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME protocol_provenance
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/ci/check_protocol.py")
    add_executable(host_dependencies_test tests/host_dependencies.cpp)
    target_link_libraries(host_dependencies_test PRIVATE panthera_protocol panthera_eigen acados)
    add_test(NAME host_dependencies COMMAND host_dependencies_test)
endif()
