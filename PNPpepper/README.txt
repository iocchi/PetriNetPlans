==================================
   PNP library for Pepper robot
         Luca Iocchi 2017
      iocchi@dis.uniroma1.it
==================================

1. Install naoqi C++ SDK and create a qi workspace

   http://doc.aldebaran.com/2-5/dev/cpp/install_guide.html#cpp-install-guide


2. Compiling
	
	* Follow instructions to compile PNP in PNP folder

	* Compile PNPpepper stuff:

	$ cd <PATH_TO>/PetriNetPlans/PNPpepper
	$ qibuild configure -c linux64 --worktree <qi_ws>
	$ qibuild make -c linux64 --worktree <qi_ws>
	$ qibuild configure -c pepper --worktree <qi_ws>
	$ qibuild make -c pepper --worktree <qi_ws>


3. Local test

	- test 1
	$ build-linux64/sdk/bin/pnp_test 
	(actions are not started, so you need to use CTRL-C to stop the execution)

	- test 2
	$ <PATH_TO>/naoqi-sdk-2.5.5.5-linux64/naoqi
	$ export PEPPER_IP=localhost
	run the 3 action servers (actions/action_{A,B,C}.py)
	run 'pnp_naoqi' in the folder containing the file test.pnml 
	(actions are started and the plan terminates after execution of A,B,C)

4. Uploading and testing on the robot

	On local machine: 
	edit 'upload.sh' to match your Pepper environment and run it

	OnPepper:
	- test 1
	run 'pnp_test' in the folder containing the file test.pnml
	(actions are not started, so you need to use CTRL-C to stop the execution)

	- test 2
	run the 3 action servers (actions/action_{A,B,C}.py)
	run 'pnp_naoqi' in the folder containing the file test.pnml 
	(actions are started and the plan terminates after execution of A,B,C) 

