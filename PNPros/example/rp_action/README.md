### Link and compile rp_action package

```
cd ~/ros/catkin_ws/src
ln -s ~/src/PetriNetPlans/PNPros/example/rp_action .
ln -s ~/src/PetriNetPlans/PNPros/example/rp_action_msgs .
cd ..
catkin_make
```

### Run a sample plan

```
cd ~src/PetriNetPlans/PNPros/example/rp_action/scripts
./start_demo.sh 
./runplan.sh robot_0 <planname>
```

Plans available in ```ros_pnp\plans``` folder: ```sequence_loop```, ```sensing```, ```interrupt```, ```fork_join```.

Notes: 1) the ```obstacle``` condition can be activated by placing (drag with mouse) the red box in the simulated environment in front of the robot; 2) the ```wave``` action outputs some text on the screen.


### Stop the current plan

```
./runplan.sh robot_0 stop
```

### Run other plans

```
./runplan.sh robot_0 <planname>
```


### Quit the simulation

```
rosnode kill -a
```

