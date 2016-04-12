cmake_minimum_required(VERSION 2.6)
project (pnp) 

add_subdirectory(src)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
