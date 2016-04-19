--------------------------------------------
PNP Generator

Luca Iocchi and Laurent Jeanpierre 2015-2016
--------------------------------------------

Required libraries: boost, libxml++, pcrecpp

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
  
5) Generation of a PNP from a PRU

  $ cd test
  $ ../bin/pnpgen_pru icaps16_ex1.xml


Test: 
    Load output pnml files with Jarp.
    Unfortunately, they may not be readable, unless you move some places, transitions, labels...

