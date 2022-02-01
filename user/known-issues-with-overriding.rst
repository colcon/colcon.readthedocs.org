Known issues with Overriding Packages
=====================================

A package is said to be overridden when it exists in two or more workspaces that are active at the same time.
This page lists known issues with overridding packages.

.. contents:: Table of Contents
    :depth: 2

Overridden package's Headers found before overriding package's headers
----------------------------------------------------------------------

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
***********************

Isolated workspaces install each package to their own folder, meaning no two installed packages will have the same include directory.
This is not always possible.

Sort include directories according to the workspace order
*********************************************************

The only known implementation of sorting include directories according to workspace order is in ``catkin`` in ROS 1.
It requires all ``catkin`` packages to use CMake and old-style standard CMake variables.
Include directories are searched in workspace order as long as all packages in the overlay only find other packages using ``find_package(catkin REQUIRED COMPONENTS ...)`` and then use only ``${catkin_INCLUDE_DIRS}`` to add include directories to their targets.

Install headers to unique include directories
*********************************************

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
-----------------------------------------------------

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
*********************************************************

This means all packages that directly or indirectly depend on the overridden package must be added to the overlay.
In this example, that's just ``baz``.
The version of ``baz`` built in the overlay must be compatible with the version of ``bar`` in the overlay.


Undefined behavior when overridden package breaks ABI
-----------------------------------------------------

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
**************************************************

Review the changes between the overridden and overridding package to make sure they are ABI compatible.
`Here are some pointers for C++ <https://community.kde.org/Policies/Binary_Compatibility_Issues_With_C%2B%2B>`_.

Build everything above the overridden package from source
*********************************************************

This means all packages that directly or indirectly depend on the overridden package must be added to the overlay.
In this example, that's just ``baz``.
The version of ``baz`` built in the overlay must be compatible with the version of ``bar`` in the overlay.


Renamed or deleted Python modules still importable
--------------------------------------------------

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++


One-definition rule violations caused by static linking
-------------------------------------------------------

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++

Python entry_points duplicated
------------------------------

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++


Deleted Python entry_points still loaded
----------------------------------------

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++
