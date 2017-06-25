## How to test pnp_rosplan ##


Clone and follow the instructions to compile PNP, PNPros and PNPgen here:
  https://github.com/iocchi/PetriNetPlans
in particular do a 'make install' at the end of each in order to move necessary libs

Clone ROSPlan to your catkin_workspace and follow the instructions:
  https://github.com/KCL-Planning/ROSPlan
switch to the branch 'pnp' and compile.

Run the script in:
  $ cd scripts
  $ ./run_pnp_rosplan.sh

Everything should work fine!
