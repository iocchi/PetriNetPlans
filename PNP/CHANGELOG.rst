^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pnp
^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.11 (2016-07-22)
-------------------
* remove source tree installation of library
  as it break Debian builds
* Contributors: Marc Hanheide

0.0.10 (2016-07-20)
-------------------

0.0.9 (2016-07-20)
------------------
* Merge pull request `#1 <https://github.com/marc-hanheide/PetriNetPlans/issues/1>`_ from marc-hanheide/master
  bringing it back in line with master branch
* * merged upstream (resolved conflicts)
  * added Luca Iocchi as author in all packages
  * sanitised some package.xml and CMakeLists.txt
  * Merge branch 'master' of https://github.com/iocchi/PetriNetPlans into upstream-merge
  Conflicts:
  PNP/src/CMakeLists.txt
* added global PNP installation
* Contributors: Luca Iocchi, Marc Hanheide

0.0.8 (2016-07-20)
------------------

0.0.7 (2016-04-26)
------------------

0.0.6 (2016-04-26)
------------------

0.0.5 (2016-04-26)
------------------

0.0.4 (2016-04-26)
------------------
* removed in source lib installation as it kills the whole debian release process
* Contributors: Marc Hanheide

0.0.3 (2016-04-26)
------------------

0.0.2 (2016-04-25)
------------------
* cleaned changelogs
* Contributors: Marc Hanheide

0.0.1 (2016-04-25)
------------------
* made Marc Hanheide maintainer
* Revert "libpnp.so must be installed locally"
  This reverts commit 69d382924c8d2634647383f514fb1fa6e61fa508.
* libpnp.so must be installed locally
* added install targets
* a first catkinised version of the core libraryx
* Improved conditions parsing
* LI Modified parser to accept ((not A))
* LI improved condition evaluation, using a cache
* Merged GUI visualization of plan execution; modified PNPjarp.jar
* Fixed activePlaces bug when sending a new plan; added README in pnp_ros
* removed build folder
* Possible pnp v1.1
* realligned PNPros with the groovy version
* added sockets for visualizing in mod java gui
* Adjusted launch files in rp_action
* Fixed PNP README
* Removed space in IDENTIFIER in ConditionScanner
* removed space from IDENTIFIER in ConditionScanner
* New parser for Bison 3
* PNP parser problem with 14.04
* Updated PNP README file
* Updated PNP README file
* Other updates on cmake files
* PNP small adjustment in CMake files and README
* PNP README.txt updated
* PNP README.txt updated
* Added PNP library
* Created folders for PNP library and PNP-ROS bridge
* Contributors: Julia Nitsch, Luca Iocchi, Marc Hanheide, g-gemignani
