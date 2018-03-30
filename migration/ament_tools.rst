ament_tools
===========

The following describes the mapping of some ``ament_tools`` options and arguments to the ``colcon`` command line interface.

ament build | test
------------------

``[BASEPATH]``
  ``--base-paths BASEPATH``

``--build-space PATH``
  ``--build-base PATH``

``--install-space PATH``
  ``--install-base PATH``

``--build-tests``
  ``--cmake-args \ -DBUILD_TESTING=1``

``-s``, ``--symlink-install``
  ``--symlink-install``

``--isolated``
  The colcon option ``--merge-install`` has the inverse logic.

``--start-with PKGNAME``
  ``--package-start PKGNAME``

``--end-with PKGNAME``
  ``--package-end PKGNAME``

``--only-packages PKGNAME1 ... PKGNAMEn``
  ``--package-whitelist PKGNAME1 ... PKGNAMEn``

``--skip-packages PKGNAME1 ... PKGNAMEn``
  ``--package-blacklist PKGNAME1 ... PKGNAMEn``

``--parallel``
  colcon uses the parallel execution by default.
  To build packages sequentially use ``--executor sequential``.

ament build
-----------

``colcon build ...``

``--cmake-args -D... --``
  ``--cmake-args \ -D...``, ``--cmake-args " -D..."``
  Any CMake arguments which start with a dash need to be prefixed with a space.
  This can either be done using an escaped space or by quoting the argument with a leading space.
  The closing double dash is not necessary anymore.

``--force-cmake-configure``
  ``--cmake-force-configure``

``--use-ninja``
  ``--cmake-args \ -G Ninja``

ament test
----------

``colcon test ...``

``--ctest-args ... --``
  ``--ctest-args ...``
  Any CTest arguments which start with a dash need to be prefixed with a space (see ``--cmake-args``).

``--retest-until-fail N``
  ``--retest-until-fail N``

``--retest-until-pass N``
  ``--retest-until-pass N``

``--abort-on-test-error``
  ``--abort-on-error``

ament test_results
------------------

``colcon test-result ...``

``[BASEPATH]``
  ``--build-base BASEPATH``

``--verbose``
  ``--all``
