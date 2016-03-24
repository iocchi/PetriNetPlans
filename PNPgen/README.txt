
PNP Generator

Luca Iocchi 2015
----------------

Compile:
  $ mkdir build
  $ cd build
  $ cmake ..
  $ make
  
Run:

1) Generation of a PNP from a linear plan + execution rules

  $ cd test
  $ ../bin/pnpgen_linear icaps16_1.plan icaps16_1.er ICAPS16
  
  Test: load output file DIAG_printer.pnml with Jarp. 
  Unfortunately, it is not readable, unless you move some places, transitions, labels...
  
2) Generation of a PNP from a policy

  $ cd test
  $ ../bin/pnpgen_policy
  
  Test: load output file SimplePolicy.pnml with Jarp.

  