# README #

This folder contains scripts for Docker installation of Petri Net Plans.


## INSTALL ##

* Install [docker](www.docker.com)

    Linux version suggested. See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.


* Configure your system (host OS)

    Create a folder `$HOME/playground` that will be shared with the docker container.

        mkdir -p $HOME/playground

    This folder will contain permanent files (i.e., files that will survive docker container execution).
    Any other file saved in other folders of the docker container will be lost when the container is closed.

    If you want to use different folders, duplicate and modify the `run.bash` script with your own folders.
    Do not change `run.bash` directly.

* Build an image

    Choose the version you prefer (you can use both anyway):
    
        1604    Ubuntu 16.04 + ROS Kinetic
        1804    Ubuntu 18.04 + ROS Melodic

    Build the image

        cd docker/[1604|1804]
        ./build.bash 


* Run an image

        cd docker/[1604|1804]
        ./run.bash

* Build and run specific images

    Create a new `Dockerfile` (do not modify the existing one)

        cd docker/[1604|1804]
        ./build.bash <new_Dockerfile> <version>

        ./run.bash <version>


    These docker images use [tmux](https://github.com/tmux/tmux/wiki) as  terminal multiplexer.



* Delete an image

    Images use several GB of disk space. If you want to remove an image you are
    not using anymore, use the following commands:

        docker image ls

        REPOSITORY                TAG     IMAGE ID         ...
        image-you-want-to-delete  0.0     6b82ade82afd     ...
        
        docker image rm -f <IMAGE ID>





### Test ###

Run each set of commands in a different terminal.
If you are using `tmux` in docker (default configuration), 
use `CTRL-b c` to create new tmux windows.


* Run ROS core

        roscore


* RUN `pnp_node`

        rosrun pnp_ros pnp_node


* RUN an action server

        rosrun pnp_ros PNPActionServer.py


### Development ###

TODO


