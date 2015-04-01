cmake_minimum_required(VERSION 2.6)
project (PNPgen) 

#search for boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# I don't think I need to link against boost
# set(LIBS ${LIBS} ${Boost_LIBRARY_DIRS})

#include directory with the headers of the source files
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

#ADD_LIBRARY(pnp SHARED ${pnp_SRC})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(genpnp SHARED src/pnpgenerator.cpp)

add_executable(test_genPNP src/test_genPNP.cpp)

target_link_libraries(test_genPNP genpnp)

