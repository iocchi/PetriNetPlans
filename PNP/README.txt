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
    $ cmake ../src
    $ make install

If everything goes fine you, should find a directory lib/ containing the newly created shared object, named libpnp.so. Use the directory include/ to compile your sources that depend on PNP and lib/ to link them.

* Set up environment variables in the file .bashrc in your home directory

    export PNP_INCLUDE=<path_to_PNP>/include
    export PNP_LIB=<path_to_PNP>/lib

* Doxygen 

You can generate the reference documentation using doxygen:

$doxygen

in the directory PNP/ where the file named Doxyfile is. 


