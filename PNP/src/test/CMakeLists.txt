
add_subdirectory(learning_plan)

SET( test_SRC ${test_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/mainTest.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TestInstantiator.cpp
	PARENT_SCOPE)