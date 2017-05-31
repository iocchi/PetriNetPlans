cmake_minimum_required(VERSION 2.8)
project(test_flex)

find_package(qibuild)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

message("current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")


# Required libraries

find_package(Boost REQUIRED)

if ( CMAKE_TOOLCHAIN_FILE MATCHES "pepper" )
	message("Pepper toolchain")
	find_package(xml2 REQUIRED) # pepper
	set(LIB_PEPPER "-L${CMAKE_CURRENT_SOURCE_DIR}/build-pepper")
	set(CTC_NAOQI_DIR "/opt/Aldebaran/naoqi/ctc-linux64-atom-2.5.2.74/")
	set(LIBXML2_INCLUDE_DIR "${CTC_NAOQI_DIR}/xml2/include/libxml2/")
	set(LIBXML2_LIBRARIES "-L${CTC_NAOQI_DIR}/xml2/lib -L${CTC_NAOQI_DIR}/zlib/lib -L${CTC_NAOQI_DIR}/xz_utils/lib")
else()
	message("Linux toolchain")
	find_package(LibXml2 REQUIRED) # linux
endif()

message("Boost: ${Boost_INCLUDE_DIR}")
message("libxml2: ${LIBXML2_INCLUDE_DIR} ${LIBXML2_LIBRARIES}")

# Setting include and lib directories

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})

set(LIBD ${LIBXML2_LIBRARIES} ${LIB_PEPPER})


# Source files

SET(PNP_SRC src/conditionchecker.cpp
	src/petrinet.cpp
	src/pnp_action.cpp
	src/pnp_action_loader_txt.cpp
	src/pnp_executable.cpp
	src/pnp_executer.cpp
	src/pnp_instantiators.cpp
	src/pnp_plan.cpp
	src/utils.cpp
	src/basic_plan/xml_plan_instantiator.cpp
	src/parser/ConditionParser.cpp
	src/parser/ConditionScanner.cpp )

set(TEST_SRC src/test_pnp.cpp src/ActionProxy.cpp )


# Libraries and executables

# libpnp.so

ADD_LIBRARY(pnp SHARED ${PNP_SRC})
target_link_libraries(pnp ${LIBD})

# test_pnp

add_executable(test_pnp ${TEST_SRC})
add_dependencies(test_pnp pnp)

target_link_libraries(test_pnp ${LIBD} libpnp.so libxml2.so libz.so liblzma.so)


qi_create_bin(test_flex "src/test_flex.cpp")


