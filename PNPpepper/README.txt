==================================
   PNP library for Pepper robot
         Luca Iocchi 2017
      iocchi@dis.uniroma1.it
==================================

1. Install naoqi C++ SDK

   http://doc.aldebaran.com/2-5/dev/cpp/install_guide.html#cpp-install-guide


2. Create a qi package

	$ cd <qi_ws>
	$ cp -a <PATH_TO>/PetriNetPlans/PNPpepper .


3. Compiling

    $ cd <qi_ws>/PNPpepper
	$ qibuild configure
	$ qibuild make
	$ qibuild configure -c pepper
	$ qibuild make -c pepper


4. Uploading and testing on the robot

	On local machine: 
	edit 'upload.sh' to match your Pepper environment and run it

	On Pepper:
	run 'test_pnp' for the folder containing the file test.pnml 


