# Docker file for Petri Net Plans
# Luca Iocchi, DIAG, Sapienza University of Rome, Italy
# Version 0.1

FROM ub1604_kinetic_pnp:base


### PNP packages ###

USER robot

RUN mkdir -p $HOME/src/

# Petri Net Plans

RUN cd $HOME/src && git clone https://github.com/iocchi/PetriNetPlans

RUN cd $HOME/src/PetriNetPlans/PNP &&  mkdir -p build &&  cd build  &&  cmake ..  && make

RUN cd $HOME/src/PetriNetPlans/PNPgen &&  mkdir -p build &&  cd build  &&  cmake ..  && make

USER root

RUN cd /home/robot/src/PetriNetPlans/PNP/build && make install

RUN cd /home/robot/src/PetriNetPlans/PNPgen/build && make install

USER robot

RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/example/rp_action . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/example/rp_action_msgs . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/example/rp_pnp . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/example/rp_demo .


# Compile ROS packages

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


