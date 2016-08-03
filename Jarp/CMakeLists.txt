cmake_minimum_required(VERSION 2.6)
project (jarp) 

install(PROGRAMS
  scripts/jarp.sh
  DESTINATION bin
)

install(DIRECTORY lib DESTINATION lib/jarp)
install(FILES PNPjarp.jar DESTINATION lib/jarp)

