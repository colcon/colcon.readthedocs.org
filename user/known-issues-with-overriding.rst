Overriding Packages and Known Issues
====================================

Two or more colcon workspaces can be active at the same time.
This is called overlaying a workspace, and it's used to extend already-built workspaces with new packages.
Earlier workspaces are called underlay workspaces, and later ones are called overlay workspaces.

When a package exists in two or more workspaces that are active at the same time in the same terminal, the package in the latest overlay workspace **overrides** the same package in all underlay workspaces.
Overriding a package is used to change the version of a package used, maybe to add features or fix bugs in the package, but it may not work in all cases.
This page lists known issues and offers tips for how to avoid them.

.. contents:: Table of Contents
    :depth: 3


How to avoid most issues when overriding packages
-------------------------------------------------

Make sure the overriding package is API and ABI compatible
**********************************************************

This advice applies when overriding a package which is depended upon by one or more packages in an underlay.

The overriding package (the one in the overlay) must be API and `ABI compatible <https://stackoverflow.com/questions/2171177/what-is-an-application-binary-interface-abi>`_ with the same package in any underlay workspace.

Only override a leaf package or a leaf group of packages
********************************************************

This advice applies when you must override a package with a version that breaks API or ABI.

A **leaf package** is one that has no other packages that depend on it.
A **leaf group** of packages is a set of packages that may depend on each other, but no package outside of the group depends on one of its members.

When possible only override leaf packages.
If that is not possible, then override the leaf group starting with that package.
That means you must also override every package that depends directly or transitively on the one you actually want to override.
If there are multiple underlay workspaces, the group of overridden packages must span all of them.

For example, say there are 3 workspaces (**A**, **B** and **C**) where **C** overlays **B** which overlays **A**.
**A** contains packages ``foo`` and ``baz`` where ``baz`` depends on ``foo``.
**B** contains packages ``ping`` and ``pong`` that also depend on ``foo``.
**C** is the workspace being built, and you want to override ``foo`` with a version that changed a public API or broke ABI.
You must also override ``baz``, ``ping``, and ``pong`` in **C**.

Only override a package if the underlay containing it is an isolated workspace
******************************************************************************

This advice applies when overriding a package that contains headers.

When possible only override packages from isolated workspaces.
That means don't override packages from underlay workspaces that were built with the ``colcon build --merge-install``.

This advice is impossible if the underlay workspace was built by someone else.
`ROS <https://www.ros.org/>`_ users should be aware that binary packages installed to ``/opt/ros/...`` are effectively in a merged workspace.

Use only packages in the underlay that install their headers to a unique directory
**********************************************************************************

This advice applies when overriding a package that contains headers and the underlay is a merged workspace.

Means you'd pass `-I underlay/foo/include/foo`, NOT `-I underlay/foo/include`.

Make all packages in the overlay depend on packages from the overlay before packages from the underlay 
******************************************************************************************************

This advice applies when overriding a package that contains headers, the underlay is a merged workspace, and an overridden package installs its headers to a directory shared with another package.

`ROS 1 <https://www.ros.org/>`_ users should be aware that ``find_package(catkin REQUIRED COMPONENTS ...)`` does this, but all packages in the overlay must use it to work.

`CMake <https://cmake.org/>`_ users should be aware that ordering include directories according to the workspace order can be impractical when using modern CMake targets.

Avoid renaming or removing top-level Python modules
***************************************************

Top level modules can still be imported.
Stuff in underlay may try to impor the old module

Avoid renaming or removing Python entry points
**********************************************

Entrypoints are collected globally, so overriding only

When writing software that uses Python Entrypoints, ignore duplicates or prefer the last one
********************************************************************************************

Known issues with Overriding Packages
-------------------------------------

Include Directory Search Order Problem
**************************************
An overridden package's headers might be included instead of the overriding package's.
This may present as: no issues, or a failure to build, or undefined behavior at runtime.
If the wrong headers are found the the behavior depend on the differences between the overriding and overridden package's headers.

Example:
++++++++
Consider an overlay containing package ``foo`` and ``bar``, and an underlay containing ``bar`` and ``baz``.
``foo`` depends on ``bar`` and ``baz``.
The underlay is a merged workspace, and both the overriden ``bar`` and ``baz`` install their headers to a directory called ``include/``.
If any libraries or executables in ``foo`` are configured to search for headers in ``baz``'s include directory first, then headers from overridden ``bar`` will also be found first.
This can cause a failure to build ``foo``, or undefined behavior at runtime when using ``foo`` depending on the differences between overridden ``bars``'s and overriding ``bar``'s headers.

When it can happen
++++++++++++++++++

* The underlay workspace is a merged workspace
* The overridden package installs header files (C/C++)
* The overriding package's headers are different from the overridden package's
* At least one more package in the underlay
   * is not overridden
   * installs headers to the same directory as the overridden package (such as ``include``)
* A package in the overlay depends on both the package being overridden and the mentioned additional package in the underlay.

How to avoid it
+++++++++++++++


Use isolated workspaces
^^^^^^^^^^^^^^^^^^^^^^^

Isolated workspaces install each package to their own folder, meaning no two installed packages will have the same include directory.
This is not always possible.

Sort include directories according to the workspace order
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The only known implementation of sorting include directories according to workspace order is in ``catkin`` in ROS 1.
It requires all ``catkin`` packages to use CMake and old-style standard CMake variables.
Include directories are searched in workspace order as long as all packages in the overlay only find other packages using ``find_package(catkin REQUIRED COMPONENTS ...)`` and then use only ``${catkin_INCLUDE_DIRS}`` to add include directories to their targets.

Install headers to unique include directories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If every package in the underlay installs their headers to unique directories, then packages in the overlay cannot accidentally find headers when depending on other packages in the underlay.

Consider a CMake package that has a ``CMakeLists.txt`` and a folder ``include/`` containing headers.
It can avoid its headers being found accidentally by installing its headers to ``include/${PROJECT_NAME}``.

.. code-block:: CMake

  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

All targets in your project should use the following to make it aware of the unique directory when exported.

.. code-block:: CMake

    target_include_directories(some_target_name_here INTERFACE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


Undefined behavior when overridden package breaks API
*****************************************************

Consider an overlay containing ``bar``, and an underlay containing ``bar`` and ``baz``.
``baz`` depends on ``bar``.
If ``bar`` in the overlay changed an API used by ``baz``, then it is undefined what will happen when ``baz`` is used at runtime.

When it applies
+++++++++++++++

* The overriding package removed or changed APIs compaired to the overridden package
* A package in the underlay depends on the overridden package

How to avoid it
+++++++++++++++

Build everything above the overridden package from source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This means all packages that directly or indirectly depend on the overridden package must be added to the overlay.
In this example, that's just ``baz``.
The version of ``baz`` built in the overlay must be compatible with the version of ``bar`` in the overlay.


Undefined behavior when overridden package breaks ABI
*****************************************************

Consider an overlay containing ``bar``, and an underlay containing ``bar`` and ``baz``.
``baz`` depends on ``bar``.
If ``bar`` in the overlay changed ABI, then it is undefined what will happen when ``baz`` is used at runtime.

When it applies
+++++++++++++++

* The overridden package uses a compiled language (C/C++, etc)
* The overriding package is ABI incompatible with the overridden one.

How to avoid it
+++++++++++++++

Make sure the overriding package is ABI compatible
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Review the changes between the overridden and overridding package to make sure they are ABI compatible.
`Here are some pointers for C++ <https://community.kde.org/Policies/Binary_Compatibility_Issues_With_C%2B%2B>`_.

Build everything above the overridden package from source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This means all packages that directly or indirectly depend on the overridden package must be added to the overlay.
In this example, that's just ``baz``.
The version of ``baz`` built in the overlay must be compatible with the version of ``bar`` in the overlay.


Renamed or deleted Python modules still importable
**************************************************

Consider an overlay containing a Python package ``pyfoo`` and an underlay containing a Python package ``pyfoo``.
``pyfoo`` in the underlay installs the Python modules ``foo``, ``foo.bar``, and ``baz``.
``pyfoo`` in the overlay installs only the Python modules ``foo``.

When the overlay is active, users will still be able to import ``baz`` from the underlay version of ``pyfoo``
However, they will not be able to import ``foo.bar`` because Python will find the ``foo`` package in overlay first, and that one does not contain ``bar``.

When it applies
+++++++++++++++

* The package being overridden is a Python package
* The overridden package installs top level modules not present in the overridding package

How to avoid it
+++++++++++++++

There's not yet a workaround.

One-definition rule violations caused by static linking
*******************************************************

Consider an overlay containing packages ``foo`` and ``bar``, and an underlay containing packages ``bar`` and ``baz``.
``foo`` depends on ``bar`` and ``baz``.
``baz`` depends on ``bar`` and has a library that statically links to another library in ``bar``.
``foo`` has a library depending on both the mentioned library in ``baz`` and in ``bar``.

When ``foo`` is used there are two definitions for symbols from ``bar``: the ones from the underlay version of ``bar`` via ``baz``, and the one from the overlay version of ``bar``.
At runtime, the implmementations from the underlay version may be used.

When it applies
+++++++++++++++

* a package in the underlay statically links to the overridden package
* a package in the overlay depends on the overriding package and the ather package in the underlay

How to avoid it
+++++++++++++++

Build everything above the overridden package from source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This means all packages that directly or indirectly depend on the overridden package must be added to the overlay.
In this example, that's just ``baz``.

Python entry_points duplicated
******************************

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++


Deleted Python entry_points still loaded
****************************************

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++
