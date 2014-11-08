PNP library
Compilation and Installation
----------------------------

* Install dependent libraries

    cmake
    libxml2 (both binary and -dev package)
    flex

* Compile the code

    $ cd PNP
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make install

If everything goes fine you, should find a directory lib/ containing the newly created shared object, named libpnp.so. Use the directory include/ to compile your sources that depend on PNP and lib/ to link them.

* Set up environment variables in the file .bashrc in your home directory

    export PNP_INCLUDE=<path_to_PNP>/include
    export PNP_LIB=<path_to_PNP>/lib

In order to have these variables set up for next steps using PNP,
do not forget to actually set these variables.
For example, open a new shell or set these variables in your current shell as well.
    
* Doxygen 

You can generate the reference documentation using doxygen:

    $ doxygen

in the directory PNP/ where the file named Doxyfile is. 


For additional information about PNP parser see the txt file in src/parser folder.


