# PNP library

## Build and Install


### Install dependent libraries

```diff
- Note: not needed if you are using the docker image. -
```

    sudo apt-get install g++ cmake libxml2 libxml2-dev flex 

### Compile the code

```diff
- Note: not needed if you are using the docker image. -
```
    
    cd PNP
    mkdir build
    cd build
    cmake ..
    make

### Install

```diff
- Note: not needed if you are using the docker image. -
```
    
Choose either global or local install. Choose local install if you plan to develop with PNP and you want to keep several versions on the same machine.

Global install (`/usr/local/lib`)

    sudo make install

Local install in any `<lib_folder>` and set up environment variables

    mkdir <lib_folder>
    cd <lib_folder>
    ln -s <path_to>/PetriNetPlans/PNP/build/src/libpnp.so .

    export PNP_HOME=$HOME/src/PetriNetPlans
    export PNP_INCLUDE=$PNP_HOME/PNP/include
    export PNP_LIB=<lib_folder>



### Doxygen 

You can generate the reference documentation using doxygen:

    doxygen

in the directory `PNP` where the file named `Doxyfile` is. 


