cmake_minimum_required(VERSION 2.8)
project(PNPnaoqi)

find_package(qibuild)

message("Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

message("Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")


# Source files

SET(PNP_SRC 
	../PNP/src/conditionchecker.cpp
	../PNP/src/petrinet.cpp
	../PNP/src/pnp_action.cpp
	../PNP/src/pnp_action_loader_txt.cpp
	../PNP/src/pnp_executable.cpp
	../PNP/src/pnp_executer.cpp
	../PNP/src/pnp_instantiators.cpp
	../PNP/src/pnp_plan.cpp
	../PNP/src/utils.cpp
	../PNP/src/basic_plan/xml_plan_instantiator.cpp
	../PNP/src/parser/ConditionParser.cpp
	../PNP/src/parser/ConditionScanner.cpp 
    ../PNP/src/connection_observer.cpp
)

SET(PNPGEN_SRC 
	../PNPgen/src/pnpgen_condplan_translator.cpp
	../PNPgen/src/pnpgenerator.cpp 
    ../PNPgen/src/pnp_translator/digraph_transl.cpp
)


set(PNP_TEST_SRC 
	src/pnp_test.cpp 
	src/ActionProxy.cpp 
)

set(PNP_NAOQI_SRC 
	src/pnp_naoqi.cpp 
	src/NaoqiActionProxy.cpp 
)

# Libraries and executables

include_directories( ../PNP/include ../PNPgen/src ../PNPgen/include include /usr/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

qi_create_lib(
  pnp
  SHARED
  ${PNP_SRC}
)
qi_use_lib( pnp xml2 )
qi_stage_lib( pnp )


qi_create_bin( pnp_test ${PNP_TEST_SRC} )

qi_use_lib(
  pnp_test
  pnp )

qi_create_bin( pnp_naoqi ${PNP_NAOQI_SRC} )

qi_use_lib( pnp_naoqi
  pnp 
  ALCOMMON ALPROXIES ALTHREAD 
)

qi_create_bin( pnpgen_translator ${PNPGEN_SRC} )

qi_use_lib( pnpgen_translator
  boost_regex
)
