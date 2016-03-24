
PNP Generator

Luca Iocchi 2015
----------------

Compile:
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make
  
Run:

1) Generation of a PNP from a linear plan

  $ cd test
  $ ../bin/pnpgen_linear DIAG_printer.plan

2) Generation of a PNP from a linear plan and execution rules

  $ cd test
  $ ../bin/pnpgen_linear DIAG_printer.plan DIAG_printer.er
 
3) Generation of a PNP from a policy

  $ cd test
  $ ../bin/pnpgen_policy
  
4) Generation of a PNP from a policy and execution rules
 
  $ cd test
  $ ../bin/pnpgen_policy DIAG_printer.er
  


Test: 
    Load output files pnml with Jarp. 
    Unfortunately, they may not be readable, unless you move some places, transitions, labels...