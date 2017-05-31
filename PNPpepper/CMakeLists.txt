cmake_minimum_required(VERSION 2.8)
project(PNPpepper)

find_package(qibuild)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")


# Source files

SET(PNP_SRC 
	src/conditionchecker.cpp
	src/petrinet.cpp
	src/pnp_action.cpp
	src/pnp_action_loader_txt.cpp
	src/pnp_executable.cpp
	src/pnp_executer.cpp
	src/pnp_instantiators.cpp
	src/pnp_plan.cpp
	src/utils.cpp
    src/connection_observer.cpp
	src/basic_plan/xml_plan_instantiator.cpp
	src/parser/ConditionParser.cpp
	src/parser/ConditionScanner.cpp 
)

set(TEST_SRC 
	src/test_pnp.cpp src/ActionProxy.cpp )


# Libraries and executables

include_directories( include )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

qi_create_lib(
  pnp
  SHARED
  ${PNP_SRC}
)
qi_use_lib( pnp xml2 )
qi_stage_lib( pnp )


qi_create_bin( pnp_test ${TEST_SRC})

qi_use_lib(
  pnp_test
  pnp ALCOMMON ALPROXIES ALTHREAD )




