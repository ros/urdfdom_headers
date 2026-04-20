^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdfdom_headers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-04-20)
-----------
* Update lower, upper, effort, and velocity default joint limits (`#95 <https://github.com/ros/urdfdom_headers/issues/95>`_)
* Clean up declaration of ModelInterface's SharedPtrs (`#99 <https://github.com/ros/urdfdom_headers/issues/99>`_)
* Extend ``JointLimits`` class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`_)

* Contributors: Aarav Gupta, Alejandro Hernández Cordero, Robert Haschke, Sai Kishor Kothakota


2.1.2 (2026-04-18)
-----------
* Revert "Extend JointLimits class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`_)"
  This was a breaking change that will be released in 3.0.0
* Contributors: Steve Peters

2.1.1 (2026-04-15)
------------------

* Clean up declaration of ModelInterface's SharedPtrs (`#99 <https://github.com/ros/urdfdom_headers/issues/99>`_)
* Revert cleanup of ModelInterface's SharedPtrs (`#33 <https://github.com/ros/urdfdom_headers/issues/33>`_)
* Revert fix for assumption that CMAKE_INSTALL_*DIR paths are relative (`#90 <https://github.com/ros/urdfdom_headers/issues/90>`_) (`#97 <https://github.com/ros/urdfdom_headers/issues/97>`_)
* Clean up declaration of ModelInterface's SharedPtrs (`#33 <https://github.com/ros/urdfdom_headers/issues/33>`_)
* Fix assumption that CMAKE_INSTALL_*DIR paths are relative (`#90 <https://github.com/ros/urdfdom_headers/issues/90>`_)
* Extend ``JointLimits`` class to include acceleration, deceleration and jerk limits (`#83 <https://github.com/ros/urdfdom_headers/issues/83>`_)

* Contributors: Alejandro Hernández Cordero, Michal Sojka, Robert Haschke, Sai Kishor Kothakota

2.1.0 (2026-01-31)
------------------
* Add support for capsule geometry type (`#94 <https://github.com/ros/urdfdom_headers/issues/94>`_)
* Contributors: Sai Kishor Kothakota

2.0.2 (2025-12-19)
------------------
* 2.0.2
* Read cmake version from package.xml (`#92 <https://github.com/ros/urdfdom_headers/issues/92>`_)
  Use regex to match version string.
  Copied from `ros/urdfdom#236 <https://github.com/ros/urdfdom/issues/236>`_.
* Contributors: Steve Peters

2.0.1 (2025-11-28)
------------------
* quaternions in urdf (PR 51 new attempt) + bump version (`#77 <https://github.com/ros/urdfdom_headers/issues/77>`_)
* fix cmake deprecation (`#89 <https://github.com/ros/urdfdom_headers/issues/89>`_)
  cmake version < then 3.10 is deprecated
* Contributors: Guillaume Doisy, mosfet80

2.0.0 (2025-07-01)
------------------
* 2.0.0
* Remove all dependencies from the package.xml. (`#88 <https://github.com/ros/urdfdom_headers/issues/88>`_)
  This package does not have any header dependencies, so
  we don't need any of them here.
* Fix package.xml deps to use vendored packages (`#87 <https://github.com/ros/urdfdom_headers/issues/87>`_)
* add package.xml file from release repository (`#85 <https://github.com/ros/urdfdom_headers/issues/85>`_)
* Removed headers, implementation was deprecated and removed (`#86 <https://github.com/ros/urdfdom_headers/issues/86>`_)
* Remove CODEOWNERS. (`#81 <https://github.com/ros/urdfdom_headers/issues/81>`_)
  It is outdated and no longer serving its intended purpose.
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Jorge J. Perez, Lucien Morey

1.1.2 (2024-09-11)
------------------
* 1.1.2
* avoid embedding the absolute install prefix in the artifacts (`#79 <https://github.com/ros/urdfdom_headers/issues/79>`_)
* Contributors: Marco A. Gutierrez, Øystein Sture

1.1.1 (2023-12-26)
------------------
* 1.1.1
* Update CMakeLists.txt (`#76 <https://github.com/ros/urdfdom_headers/issues/76>`_)
  cmake minimum version requirement for rolling are v3.8
* Install pkg-config .pc file also on Windows/MSVC (`#72 <https://github.com/ros/urdfdom_headers/issues/72>`_)
* Add Chris Lalancette to CODEOWNERs file (`#74 <https://github.com/ros/urdfdom_headers/issues/74>`_)
* Contributors: Audrow Nash, Chris Lalancette, Silvio Traversaro, mosfet80

1.1.0 (2022-05-10)
------------------
* Bump to 1.1.0
* Contributors: Audrow Nash

1.0.6 (2022-03-28)
------------------
* Bump to 1.0.6
* Install headers to include/${PROJECT_NAME} (`#71 <https://github.com/ros/urdfdom_headers/issues/71>`_)
  * Install headers to include/${PROJECT_NAME}
  * Rename files to make this way easier
  * use CMAKE_INSTALL_INCLUDEDIR
* Replace float compare to 0 with <= compare (`#68 <https://github.com/ros/urdfdom_headers/issues/68>`_)
  * Replace float compare to 0 with <= compare
* Upgrade cmake minimum version to 2.8.12 (`#67 <https://github.com/ros/urdfdom_headers/issues/67>`_)
* Contributors: Audrow Nash, Shane Loretz, Stephen Brawner

1.0.5 (2020-05-08)
------------------
* bump to 1.0.5 (`#64 <https://github.com/ros/urdfdom_headers/issues/64>`_)
* install pkgconfig file into correct path (`#58 <https://github.com/ros/urdfdom_headers/issues/58>`_)
* export modern CMake interface target (`#61 <https://github.com/ros/urdfdom_headers/issues/61>`_)
* avoid exporting absolute path (`#60 <https://github.com/ros/urdfdom_headers/issues/60>`_)
* Contributors: Dirk Thomas, Steven Peters, Victor Mataré

1.0.4 (2019-05-20)
------------------
* bump to 1.0.4 (`#53 <https://github.com/ros/urdfdom_headers/issues/53>`_)
* Add missing include, otherwise dependent packages fail to find the function when compiling. (`#52 <https://github.com/ros/urdfdom_headers/issues/52>`_)
* Contributors: Raúl Domínguez, Steven Peters

1.0.3 (2019-01-08)
------------------
* Fix version number, bump to 1.0.3 (`#50 <https://github.com/ros/urdfdom_headers/issues/50>`_)
  The version number was not incremented for the 1.0.2 release, so bump to 1.0.3 to make a release with proper version numbers.
* Contributors: Steven Peters

1.0.2 (2018-11-15)
------------------
* Cast double to float (`#48 <https://github.com/ros/urdfdom_headers/issues/48>`_)
* Add range check for color components being within [0, 1]
  this solves the potential undefined behaviour in implicit conversion if a color component should
  be greater than the maximum float but smaller than the maximum double and allows us
  to reuse strToDouble safely
* use helper function strToDouble for locale independent parsing of colors
* Contributors: Shane Loretz, Simon Schmeisser, Simon Schmeisser (isys vision)

1.0.1 (2018-02-21)
------------------
* Use stringstream instead of stod to work around locale issues. (`#42 <https://github.com/ros/urdfdom_headers/issues/42>`_)
  * Use stringstream instead of stod to work around locale issues.
  * Use imbue to always use the "classic" locale.
  * Make a strToDouble facility in the utils.h header.
  This will be useful elsewhere (i.e. urdfdom).
  * Add some additional includes.
  * Add in an actual error message to the strToDouble throw.
* urdf_model: don't use or define M_PI (`#38 <https://github.com/ros/urdfdom_headers/issues/38>`_)
* world.h: remove initXml function, tinyxml include (`#39 <https://github.com/ros/urdfdom_headers/issues/39>`_)
  * world.h: forward declare tinyxml and don't include
  The urdf::World class uses a pointer to a
  tinyxml symbol in the initXml function,
  but we don't want to export that dependency,
  so just forward declare the symbol since it's a pointer.
  * Remove the unimplemented initXml method.
  This allows us to remove tinyxml.h from the header file.
* correct spelling mistake in README.md (`#40 <https://github.com/ros/urdfdom_headers/issues/40>`_)
* Add CMake uninstall target (`#31 <https://github.com/ros/urdfdom_headers/issues/31>`_)
* Contributors: Chris Lalancette, Edward Betts, Jeongseok Lee, Steven Peters

1.0.0 (2016-07-20)
------------------
* Merge pull request `#23 <https://github.com/ros/urdfdom_headers/issues/23>`_ from scpeters/unboost
  Use c++11 instead of boost, bump version to 1.0.0
* Replace boost::lexical_cast with std::sto[df]
* Replace boost::split with urdf::split_string
  Added in urdf_model/utils.h
  Licensed to OSRF not Willow Garage
* Use std pointers instead of boost pointers
* Bump version to 1.0.0
* Contributors: Steven Peters

0.4.2 (2016-07-05)
------------------
* 0.4.2 version bump
* Merge pull request `#26 <https://github.com/ros/urdfdom_headers/issues/26>`_ from j-rivero/fix_install_path
  Install cmake modules into arch dependent path
* Install cmake modules into arch dependent path
  cmake auto generates code for the -version.cmake file which
  includes a check to test if the arch used to build the software
  is the same than the one in installed system.
* Merge pull request `#25 <https://github.com/ros/urdfdom_headers/issues/25>`_ from j-rivero/fix_version
  Fix version. Current is 0.4.1
* Fix version. Current is 0.4.1
* Contributors: Jackie Kay, Jose Luis Rivero, Steven Peters

0.4.1 (2016-02-13)
------------------
* Merge pull request `#21 <https://github.com/ros/urdfdom_headers/issues/21>`_ from scpeters/remove_reinterpret_pointer_cast
  remove urdf::reinterpret_pointer_cast
* remove urdf::reinterpret_pointer_cast
  I don't know if it's used anywhere
  and there's no std:: version of the function
  so let's just redact it.
* Contributors: Ioan A Sucan, Steven Peters

0.4.0 (2015-12-09)
------------------
* Merge pull request `#19 <https://github.com/ros/urdfdom_headers/issues/19>`_ from traversaro/master
  Fix handling rpy with pitch equal to +/-M_PI/2
* Merge pull request `#17 <https://github.com/ros/urdfdom_headers/issues/17>`_ from scpeters/cmake_version_file
  Install cmake version config file
* Fix missing include in urdf_model/types.h
  Include boost/pointer_cast.hpp since we
  are using the pointer_cast functions.
  Fixes `#66 <https://github.com/ros/urdfdom_headers/issues/66>`_.
* Fix handling rpy with pitch equal to +/-M_PI/2
* Bump minor version to 0.4
  So that new pointer typedefs can be found
* Install cmake version config file
  Uses SameMajorVersion matching criteria
* Merge pull request `#12 <https://github.com/ros/urdfdom_headers/issues/12>`_ from traversaro/patch-1
  [joint] reset `JointMimic` pointer on clear
* Merge pull request `#7 <https://github.com/ros/urdfdom_headers/issues/7>`_ from traversaro/master
  Fixes for compiling in Windows
* Remove repeated <cmath> inclusion
* Merge pull request `#16 <https://github.com/ros/urdfdom_headers/issues/16>`_ from scpeters/geometry_typedefs
  Pointer typedefs for simple geometry and urdf::*_pointer_cast function
* [cmake] modify install directory of .cmake files
  In Windows find_package(package) finds the PackageConfig.cmake following a set of path patterns [1]:
  * <prefix>/
  * <prefix>/(cmake|CMake)/
  * <prefix>/<name>*/
  * <prefix>/<name>*/(cmake|CMake)/
  This commit modifies the CMake config installation to install them in CMAKE_INSTALL_PREFIX/CMake on Windows (see for reference [2]).
  [1] : https://cmake.org/cmake/help/v3.0/command/find_package.html
  [2] : https://github.com/robotology/ycm/blob/master/modules/InstallBasicPackageFiles.cmake#L175
* only define round in VS <= 2012
* fix #ifndef _USE_MATH_DEFINES
* fixed cmake logic
* Merge branch 'master' of https://github.com/ros/urdfdom_headers
  Conflicts:
  CMakeLists.txt
  urdf_model/include/urdf_model/pose.h
* model.h remove unneeded boost include
* Add *_pointer_cast functions to urdf namespace
* Add typedefs for other geometry classes
  Box, Cylinder, Mesh, Sphere
* Create macro for pointer typedefs
* Merge pull request `#15 <https://github.com/ros/urdfdom_headers/issues/15>`_ from scpeters/sensor_boost_includes
  sensor.h: remove boost smart pointer includes
* sensor.h: remove boost smart pointer includes
* Merge pull request `#13 <https://github.com/ros/urdfdom_headers/issues/13>`_ from scpeters/pointer_typedef
  Create typedefs for shared pointers
* Merge pull request `#14 <https://github.com/ros/urdfdom_headers/issues/14>`_ from scpeters/temp_files_in_build
  Generate temp file in build folder
* Generate temp file in build folder
  Generate the pkg-config and cmake config files
  in the build folder instead of the source tree.
* use pointer typedefs instead of boost
* Add pointer typedefs in types.h for each folder
* [joint] reset `JointMimic` point on clear
  For consistency with other attributes.
* Merge pull request `#11 <https://github.com/ros/urdfdom_headers/issues/11>`_ from vrabaud/master
  fixes degeneracies when doing atan2(0,0)
* fixes degeneracies when doing atan2(0,0)
  This fixes the unit test in urdfdom at https://github.com/ros/urdfdom/issues/49
* Merge pull request `#10 <https://github.com/ros/urdfdom_headers/issues/10>`_ from scpeters/fixes_from_sdformat
  Fixes from sdformat
* Cast to float instead of double
  Also comment out unused exception variables
* Push fixes from sdformat upstream
  Use cmath instead of math.h
  Fix typo in a comment
* Merge pull request `#8 <https://github.com/ros/urdfdom_headers/issues/8>`_ from davetcoleman/patch-1
  Fixed file name for proper markup formatting
* Fixed file name for proper markup formatting
* Fixes for compiling in Windows
* Merge pull request `#5 <https://github.com/ros/urdfdom_headers/issues/5>`_ from davetcoleman/readme_improvements
  Improved README and explained how to wget package.xml
* Fix
* Improved README and explained how to wget package.xml
* Merge pull request `#4 <https://github.com/ros/urdfdom_headers/issues/4>`_ from dirk-thomas/master
  fix INCLUDE_DIRS variable when path contains spaces
* fix INCLUDE_DIRS variable when path contains spaces
* Contributors: Dave Coleman, Dirk Thomas, Ioan A Sucan, Jackie Kay, Jeongseok Lee, Silvio Traversaro, Steven Peters, Vincent Rabaud, hsu

0.3.0 (2014-02-15)
------------------
* remove functionality that was deprecated since ros hydro
* Merge pull request `#2 <https://github.com/ros/urdfdom_headers/issues/2>`_ from isucan/master
  add visual and collision names
* add visual and collision names
* fix `#41 <https://github.com/ros/urdfdom_headers/issues/41>`_
* Fix comment for collision & visual arrays
* Added tag 0.2.3 for changeset d7757a5edf3a
* Contributors: Ioan A Sucan, Ioan Sucan, isucan

0.2.3 (2013-06-18)
------------------
* merge
* removing some of the deprecation warnings since too many warnings are generated
* update comments
* update comments
* re-add clear() calls for deprecated members
* more deprecation warnings
* deprecating visual_groups, collision_groups
* remove commented code
* Merged in issue_1 (pull request `#1 <https://github.com/ros/urdfdom_headers/issues/1>`_: fix LICENSE file per issue `#1 <https://github.com/ros/urdfdom_headers/issues/1>`_)
* fix LICENSE file per issue `#1 <https://github.com/ros/urdfdom_headers/issues/1>`_
* Added tag 0.2.2 for changeset f8bcb86e7766
* Contributors: Ioan Sucan, John Hsu

0.2.2 (2012-12-22)
------------------
* add missing license headers
* Added tag 0.2.1 for changeset 62b843be1542
* Contributors: Ioan Sucan, John Hsu

0.2.1 (2012-10-02)
------------------
* update comments
* update world
* move twist from urdf_model_state to urdf_model
* Added tag 0.2.0 for changeset 0a1404c64b62
* Contributors: Ioan Sucan, John Hsu

0.2.0 (2012-06-28)
------------------
* removing unimplemented functions + fixing bugs
* remove dependency on tinyxml.h, forward declare TiXmlElement
* Added tag 0.1.5 for changeset 7382afda6474
* Contributors: Ioan Sucan, hsu

0.1.5 (2012-06-25 14:57)
------------------------
* removing parseVisualSensor from class
* Added tag 0.1.4 for changeset a5248f8f4895
* Contributors: Ioan Sucan, hsu

0.1.4 (2012-06-25 11:26)
------------------------
* simplifications to build system + add cmake-config file
* Added tag 0.1.3 for changeset 666374321561
* Contributors: Ioan Sucan

0.1.3 (2012-06-22 22:46)
------------------------
* simplify build system
* Added tag 0.1.0 for changeset 960abc214d83
* Contributors: Ioan Sucan

0.1.0 (2012-06-22 14:38)
------------------------
* update README
* copying headers from urdfdom
* Contributors: John Hsu
