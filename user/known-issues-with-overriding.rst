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


How to avoid most issues when overridding
-----------------------------------------

Here is general advice describes how to avoid most issues.
It has been simplified for ease of use, but if it is too restrictive for your case, then see the known issue descriptions for more complex advice.

Use isolated workspaces
***********************

Don't use the ``--merge-install`` option if you suspect you will override a package in this workspace in the future.
Merged workspaces can have issues include directories when overriding packages from them.

If the underlay is a merged workspace, only override a package if it installs its headers to a unique directory
***************************************************************************************************************

If you must override a package in a merged underlay workspace, only do so if it installs its headers to a unique directory.
You can figure this out by looking at the compile options needed to build a package.
If the compiler needs ``--isystem <prefix>/include`` or ``-I<prefix>/include`` (where ``<prefix>`` is the path to the root of the workspace), then don't override the package.

When overriding a non-leaf package, override everything that depends on it
**************************************************************************

A **leaf package** is one that has no other packages that depend on it.

When possible only override leaf packages.
If you must override a non-leaf package then override every package that depends directly or transitively on the one you actually want to override.
If there are multiple underlay workspaces, the group of overridden packages must span all of them.

For example, say there are 3 workspaces (**A**, **B** and **C**) where **C** overlays **B** which overlays **A**.
**A** contains packages ``foo`` and ``baz`` where ``baz`` depends on ``foo``.
**B** contains packages ``ping`` and ``pong`` that also depend on ``foo``.
**C** is the workspace being built, and you want to override ``foo`` with a version that changed a public API or broke ABI.
You must also override ``baz``, ``ping``, and ``pong`` in **C**.

If other issues prevent you from overriding one of these packages, then don't override any of them.

Only override a Python package with a version that has identical entry point specifications
*******************************************************************************************

Only override packages that use dynamic linking
***********************************************

How to make your package easy to override
-----------------------------------------

Here is general advice to make it easier for users to override your package.

Install your package's headers to a unique include directory
************************************************************

Install your packages headers to a unique folder rather than ``<prefix>/include``.

Consider a CMake package that has a ``CMakeLists.txt`` and a folder ``include/`` containing headers.
It can avoid its headers being found accidentally when it is overridden by installing its headers to ``include/${PROJECT_NAME}``.

.. code-block:: CMake

  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

All targets in your project should use the following to make it aware of the unique directory when exported.

.. code-block:: CMake

    target_include_directories(some_target_name_here INTERFACE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

Dynamically link to libraries outside your package
**************************************************

If your package ``foo`` statically links to ``libbar.lib`` from package ``bar``, then users can't override ``bar`` without also overriding yours.
Prefer dynamic linking to ``libbar.so`` instead.

Similarly, consider not providing static libraries so that other packages can't statically link to yours.

All Known issues
----------------

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

Only override packages that install headers to unique include directories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If every package in the underlay installs their headers to unique directories, then packages in the overlay cannot accidentally find headers when depending on other packages in the underlay.


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

Python entry_points are duplicated
**********************************

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++


Deleted Python entry_points may still be loaded
***********************************************

When it applies
+++++++++++++++

How to avoid it
+++++++++++++++
