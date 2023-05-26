
# Petri Net Plans

This repository contains Petri Net Plans (PNP) library, PNP ROS bridge and some sample applications.
More information on the [PNP web site](http://pnp.diag.uniroma1.it). PNP component is also a component of
[AI-on-Demand platform](https://www.ai4europe.eu/).

This master branch contains up-to-date development of main PNP library, other utilities for plan generation, bridges with ROS and NAOqi and examples.

PNP is licensed under GPL v3.

## Content of this repository

* PNP - main library
* PNPgen - library and tools for PNP generation
* PNPros - ROS bridge
* PNPnaoqi - NAOQi bridge
* Jarp - PNP GUI editor
* pyPNP - Python interface for PNP
* docker - docker installation files and scripts


# Install

## Best Option - Docker installation

Follow instructions in [docker](docker) folder.


## Manual Configuration Option - Local installation

Download the library (default folder is `$HOME/src`) and compile the components, following this order.

* Compile and install PNP (see [PNP/README.md](PNP/README.md))

* Compile and install PNPgen (see [PNPgen/README.md](PNPgen/README.md))

* Compile pnp_ros (see [PNPros/README.md](PNPros/README.md))

Libraries and packages have been tested with Ubuntu 12.04 + ROS hydro, 
Ubuntu 14.04 + ROS indigo, Ubuntu 16.04 + ROS Kinetic, Ubuntu 18.04 + ROS Melodic,

PNPnaoqi has been tested on NAOQi version 2.5.



# Examples

* [PNPros example](PNPros/examples/rp_example)

* [DIAG demo](https://github.com/iocchi/DIAG_demo)



# Video Tutorials

* [Tutorials playlist](https://www.youtube.com/playlist?list=PLyk5DuHplcDe0G4vwWnokdsVCD5hImMW-)




