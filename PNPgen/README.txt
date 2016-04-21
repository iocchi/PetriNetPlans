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

  General command:
    pnpgen_pru <PRU_file> [horizon] [discount_factor] [stop_criterion] [execution_rules_file]
    
  optional parameters:
  - horizon is the maximum number of steps to compute the policy over (default = 50)
  - discout-factor (usually denoted \gamma) is the loss of value with time (default = 0.99 - if reaching a goal in one step is rewarded 100, the same goal in two steps will be rewarded 99 only...)
  - stop_criterion (usually denoted \epsilon) is the minimal change of expected value alowed between two decision steps (default = 0 - stop if expected value does not change)
  - execution_rules_file (default = none) is a file containing execution rules to apply to the generated plan.

Test: 
    Load output pnml files with Jarp.
    Unfortunately, they may not be readable, unless you move some places, transitions, labels...

