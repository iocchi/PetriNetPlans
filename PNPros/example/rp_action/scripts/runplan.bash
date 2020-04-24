#!/bin/bash
# Use: ./runplan.bash [<robotname>] <planname>
# Example: ./runplan.bash cocktail_party
# Example: ./runplan.bash robot_0 cocktail_party

if [ "$#" == 1 ]; then

rostopic pub /planToExec std_msgs/String "data: '$1'" --once

else

rostopic pub /$1/planToExec std_msgs/String "data: '$2'" --once

fi

