add_subdirectory(env1)

SET( test_SRC ${test_SRC}
${CMAKE_CURRENT_SOURCE_DIR}/RewEnv.cpp
${CMAKE_CURRENT_SOURCE_DIR}/ActionRew.cpp
PARENT_SCOPE)
