check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/uavlink-examples-test-suite.cc
        )
endif()

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

# Add the include subdirectory for Boost.Interprocess
include_directories(
    ${Boost_INCLUDE_DIRS}/boost/interprocess/allocators
    ${Boost_INCLUDE_DIRS}/boost/interprocess/containers
    ${Boost_INCLUDE_DIRS}/boost/interprocess
)


find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
include_directories(${pybind11_INCLUDE_DIRS})

find_package(Protobuf REQUIRED)
if(NOT Protobuf_FOUND)
    message(STATUS "protobuf not found")
  return()
endif()


set(proto_source_files
    model/messages.proto
)

set(source_files
    model/uavlink.cc
    helper/uavlink-helper.cc
    model/container.cc
    model/spaces.cc
    model/opengym_env.cc
    model/opengym_interface.cc
    ${proto_source_files}
)

set(header_files
    model/uavlink.h
    helper/uavlink-helper.h
    model/container.h
    model/opengym_env.h
    model/opengym_interface.h
    model/spaces.h
)


build_lib(
    LIBNAME uavlink
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libcore} protobuf
    TEST_SOURCES test/uavlink-test-suite.cc
                 ${examples_as_tests_sources}
)

# need protobuf_generate func to generate messages
check_function_exists(protobuf_generate protobuf_generate_exists)
if(${protobuf_generate_exists})
    message(STATUS "protobuf_generate command found")
else()
  message(STATUS "protobuf_generate command not found -> use a local copy from ${CMAKE_CURRENT_SOURCE_DIR}/protobuf-generate.cmake")
  include(${CMAKE_CURRENT_SOURCE_DIR}/protobuf-generate.cmake)
endif()


protobuf_generate(
  TARGET ${libuavlink-obj}
  IMPORT_DIRS model/
  LANGUAGE cpp
  PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model
)

protobuf_generate(
  TARGET ${libuavlink-obj}
  IMPORT_DIRS model/
  LANGUAGE python
  PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model/ns3gym/ns3gym
)