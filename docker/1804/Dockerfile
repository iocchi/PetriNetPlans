# Docker file for Petri Net Plans
# Luca Iocchi, DIAG, Sapienza University of Rome, Italy
# 2020-2021

FROM iocchi/pnp:grpc


### PNP packages ###

USER robot

RUN mkdir -p $HOME/src/

# Trick to disable cache from here
#ADD http://worldclockapi.com/api/json/utc/now /tmp/time.tmp

ARG FORCEBUILD=none

RUN echo "$FORCEBUILD" > /tmp/lastdockerbuild

# Petri Net Plans

RUN cd $HOME/src && git clone --depth 1 https://github.com/iocchi/PetriNetPlans

RUN cd $HOME/src/PetriNetPlans/PNP &&  mkdir -p build &&  cd build  &&  cmake ..  && make

RUN cd $HOME/src/PetriNetPlans/PNPgen &&  mkdir -p build &&  cd build  &&  cmake ..  && make

RUN echo "export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH}:/usr/local/lib\" " >> $HOME/.bashrc

USER root

RUN cd /home/robot/src/PetriNetPlans/PNP/build && make install

RUN cd /home/robot/src/PetriNetPlans/PNPgen/build && make install

RUN cd /usr/local/bin && ln -sf /home/robot/src/PetriNetPlans/Jarp/scripts/jarp.sh .

USER robot

RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_action . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_action_msgs . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_pnp . && \
    ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_demo .


# Compile ROS packages

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make -j1"

# Remove sudo warning

RUN touch ~/.sudo_as_admin_successful


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux





