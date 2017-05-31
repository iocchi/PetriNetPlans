==================================
   PNP library for Pepper robot
         Luca Iocchi 2017
      iocchi@dis.uniroma1.it
==================================

1. Install naoqi C++ SDK and create a qi workspace

   http://doc.aldebaran.com/2-5/dev/cpp/install_guide.html#cpp-install-guide


2. Compiling

    $ cd <PATH_TO>/PetriNetPlans/PNPpepper
	$ qibuild configure --worktree <qi_ws>
	$ qibuild make --worktree <qi_ws>
	$ qibuild configure -c pepper --worktree <qi_ws>
	$ qibuild make -c pepper --worktree <qi_ws>


4. Uploading and testing on the robot

	On local machine: 
	edit 'upload.sh' to match your Pepper environment and run it

	On Pepper:
	run 'test_pnp' for the folder containing the file test.pnml 


