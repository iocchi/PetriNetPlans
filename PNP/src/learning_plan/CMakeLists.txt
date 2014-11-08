
add_subdirectory(algo)
add_subdirectory(exp)

SET( pnp_SRC ${pnp_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/learnPlan.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/learnPlanFunctors.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Learner.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Marking.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/RewardCollector.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ExpPolicy.cpp
	PARENT_SCOPE)