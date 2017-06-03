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

	* test 1
	- run 'pnp_test' in the folder containing 'plans'
	  $ build-linux64/sdk/bin/pnp_test 
	  (actions are not started, so you need to use CTRL-C to stop the execution)

	* test 2 (quick)
	- run 'run_test.sh' in the PNPpepper folder

	* test 3 (full manual instructions)
	- launch naoqi
	  $ <PATH_TO>/naoqi-sdk-2.5.5.5-linux64/naoqi
	- launch actions
	  (set PEPPER_IP=localhost if needed)
	  run the 3 action servers (actions/action_{A,B,C}.py)
	- launch pnp_naoqi
	  run 'pnp_naoqi' in the folder containing 'plans'
	- send the plan to be executed to pnp_naoqi
	  run 'run_plan.py' with the name of a plan
	  $ python run_plan.py --plan <plan_name> (e.g., 'python run_plan.py --plan test')
	  (actions are started and the plan terminates after execution, plan can be restarted 
      or stopped with 'python run_plan.py --plan stop')


4. Uploading and testing on the robot

	On local machine: 
	edit 'upload.sh' to match your Pepper environment and run it

	OnPepper:
	- test 1
	  run 'pnp_test' in the folder containing 'plans'
	  (actions are not started, so you need to use CTRL-C to stop the execution)

	- test 2
	  (as for Local test 2, without launching naoqi and setting PEPPER_IP)	

