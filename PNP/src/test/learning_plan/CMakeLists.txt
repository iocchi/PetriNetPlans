
add_subdirectory(envs)

SET( test_SRC ${test_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/MarkingTest.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/MDPAlgorithmTest.cpp
	PARENT_SCOPE)
