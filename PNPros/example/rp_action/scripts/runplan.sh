#!/bin/sh
# Use: ./runplan.sh <robotname> <planname>
# Example: ./runplan.sh diago_0 cocktail_party

rostopic pub /$1/planToExec std_msgs/String "data: '$2'" --once

