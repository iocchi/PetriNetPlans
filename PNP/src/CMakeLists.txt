
cmake_minimum_required(VERSION 2.6)
project (pnp) 


#call flex to generate the parser
execute_process(	
 		COMMAND flex
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/parser
 		INPUT_FILE  ConditionScanner.yy
 		)
 
IF(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/parser/ConditionScanner.cpp )
  MESSAGE(FATAL_ERROR "Could not generate the parser, please verify that flex is installed")
ENDIF( NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/parser/ConditionScanner.cpp )

#source file list, add new files here

SET( pnp_SRC conditionchecker.cpp
	petrinet.cpp
	pnp_action.cpp
	pnp_action_loader_txt.cpp
	pnp_executable.cpp
	pnp_executer.cpp
	pnp_instantiators.cpp
	pnp_plan.cpp
	utils.cpp
	basic_plan/xml_plan_instantiator.cpp
	parser/ConditionParser.cpp
	parser/ConditionScanner.cpp )

#comment the following lines to have traditional pnp without the learning functionality
# ----- learnpnp ------
add_subdirectory(learning_plan)
#search for boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# I don't think I need to link against boost
# set(LIBS ${LIBS} ${Boost_LIBRARY_DIRS})

# ------ learnpnp end here --------


#include directory with the headers of the source files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

ADD_LIBRARY(pnp SHARED ${pnp_SRC})

#search for libxml2
find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})
set(LIBS ${LIBS} ${LIBXML2_LIBRARIES})

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/test/cmake)
find_package(CppUnit )


#add the target for the shared library
target_link_libraries(pnp ${LIBS})

if(CPPUNIT_FOUND)
	include_directories(${CPPUNIT_INCLUDE_DIR})
	link_directories(${CMAKE_CURRENT_BINARY_DIR})

	set(test_SRC)
	add_subdirectory(test)

	add_executable(test_pnp ${test_SRC})
	

	target_link_libraries(test_pnp ${LIBS} ${CPPUNIT_LIBRARY} libpnp.so)
endif(CPPUNIT_FOUND)

#add the target to copy the library in lib
install(TARGETS pnp DESTINATION ${PROJECT_SOURCE_DIR}/../lib)
