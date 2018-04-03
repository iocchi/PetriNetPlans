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

