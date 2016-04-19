cmake_minimum_required(VERSION 2.6)
project (PNPgen) 

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#search for boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})

find_package(Libpcrecpp REQUIRED)
include_directories(${PCRE_INCLUDE_DIRS})


#include directory with the headers of the source files
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

include_directories(${PROJECT_SOURCE_DIR}/src/PRUMDP)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(pnpgen SHARED src/pnpgenerator.cpp)

ADD_LIBRARY(prumdp SHARED src/PRUMDP/MDP.cpp src/PRUMDP/MDPsolve.cpp src/PRUMDP/PRUplus.cpp src/PRUMDP/PRUprint.cpp
    src/PRUMDP/PRU2MDP.cpp src/PRUMDP/PRU2MDPprogress.cpp src/PRUMDP/PRU2MDPstateStore.cpp src/PRUMDP/PRUconstraint.cpp
)

add_executable(pnpgen_linear src/pnpgen_linear.cpp)
add_executable(pnpgen_policy src/pnpgen_policy.cpp)
add_executable(pnpgen_pru src/pnpgen_pru.cpp)

target_link_libraries(pnpgen_linear pnpgen boost_regex)
target_link_libraries(pnpgen_policy pnpgen boost_regex)
target_link_libraries(pnpgen_pru pnpgen prumdp boost_regex ${LibXML++_LIBRARIES} ${PCRE_LIBRARIES})


