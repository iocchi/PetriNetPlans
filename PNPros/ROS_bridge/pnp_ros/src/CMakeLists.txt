

#add_subdirectory(aamas12)

SET( aamas_SRC  ${aamas_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ActionProxy.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ROSInst.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ROSConds.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LearnPNP/ROSLearnInstantiator.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LearnPNP/ROSReward.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LearnPNP/World/World.cpp
PARENT_SCOPE)

