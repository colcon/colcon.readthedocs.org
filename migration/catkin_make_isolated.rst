catkin_make_isolated
====================

The following describes the mapping of some ``catkin_make_isolated`` options and arguments to the ``colcon`` command line interface.

``--source PATH``
  ``--base-paths BASEPATH``

``--build PATH``
  ``--build-base PATH``

``--devel PATH``
  colcon doesn't support the concept of a "devel" space.
  Instead you can choose the path of the devel space as the install base and perform an normal installation.

``--install-space PATH``
  ``--install-base PATH``

``--merge``
  ``--merge-install``

``--use-ninja``
  ``--cmake-args \ -G Ninja``

``--use-nmake``
  ``--cmake-args \ -G "NMake Makefiles"``

``--install``
  colcon always performs an installation.
  It doesn't support the concept of a "devel" space.

``--cmake-args ...``
  ``--cmake-args \ -D...``, ``--cmake-args " -D..."``
  Any CMake arguments which start with a dash need to be prefixed with a space.
  This can either be done using an escaped space or by quoting the argument with a leading space.
  The closing double dash is not necessary anymore.

``--force-cmake``
  ``--cmake-force-configure``

``--pkg PKGNAME1 ... PKGNAMEn``
  ``--packages-select PKGNAME1 ... PKGNAMEn``

``--from-pkg PKGNAME``
  ``--packages-start PKGNAME``

``--only-pkg-with-deps PKGNAME1 ... PKGNAMEn``
  ``--packages-up-to PKGNAME1 ... PKGNAMEn``
