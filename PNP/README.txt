PNP library
Compilation and Installation
----------------------------

* Install dependent libraries

    $ sudo apt-get install g++ cmake libboost1.54-dev libxml2 libxml2-dev flex 

* Compile the code

    $ cd PNP
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

* Install

Choose either Global or Local install. Choose Local install if you plan to develop with PNP and you want to keep several versions on the same machine.

* Global install (/usr/local/lib)

    $ sudo make install


* Local install (<path_to_PNP>/lib)

    $ cd PNP/lib
    $ ln -s ../build/src/libpnp.so .

Set up environment variables in the file .bashrc in your home directory

    export PNP_INCLUDE=<path_to_PNP>/include
    export PNP_LIB=<path_to_PNP>/lib

(open a new shell or set these variables in your current shell to use it in the current terminal session).
    



* Doxygen 

You can generate the reference documentation using doxygen:

    $ doxygen

in the directory PNP/ where the file named Doxyfile is. 


For additional information about PNP parser see the txt file in src/parser folder.


