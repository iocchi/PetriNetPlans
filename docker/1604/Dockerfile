# Docker file for Petri Net Plans
# Luca Iocchi, DIAG, Sapienza University of Rome, Italy
# Version 0.1

FROM ros:kinetic-ros-base-xenial

ARG DEBIAN_FRONTEND=noninteractive


###### USER root ######

# install libraries and ros packages 

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update

RUN apt-get install -y tmux less sudo eom

RUN apt-get install -y nano unzip iputils-ping net-tools openssh-client

RUN apt-get install -y libxml2-dev libxml++2.6-dev flex

RUN rm -rf /var/lib/apt/lists/*


# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo


###### USER robot ######

USER robot

RUN echo "set -g mouse on" > $HOME/.tmux.conf 


# Init ROS workspace

RUN mkdir -p $HOME/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd $HOME/ros/catkin_ws/src; catkin_init_workspace; cd ..; catkin_make"

RUN echo "source $HOME/ros/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

RUN rosdep update


# Set up .bashrc

RUN echo "if [ -d /usr/lib/nvidia-384/ ]; then" >> $HOME/.bashrc
RUN echo "  export PATH=\"/usr/lib/nvidia-384/bin:\${PATH}\"" >> $HOME/.bashrc
RUN echo "  export LD_LIBRARY_PATH=\"/usr/lib/nvidia-384:/usr/lib32/nvidia-384:\${LD_LIBRARY_PATH}\" " >> $HOME/.bashrc
RUN echo "fi" >> $HOME/.bashrc


### Additional packages ###

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
    ln -s $HOME/src/PetriNetPlans/PNPros/ROS_bridge/pnp_ros .


# Compile ROS packages

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"


# Set working dir and container command

WORKDIR /home/robot

CMD /usr/bin/tmux


