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
  CMake configures tests by default.
  To skip configuring tests use ``--cmake-args -DBUILD_TESTING=OFF``.

``-s``, ``--symlink-install``
  ``--symlink-install``

``--isolated``
  The colcon option ``--merge-install`` has the inverse logic.

``--start-with PKGNAME``
  ``--packages-start PKGNAME``

``--end-with PKGNAME``
  ``--packages-end PKGNAME``

``--only-packages PKGNAME1 ... PKGNAMEn``
  ``--packages-select PKGNAME1 ... PKGNAMEn``

``--skip-packages PKGNAME1 ... PKGNAMEn``
  ``--packages-skip PKGNAME1 ... PKGNAMEn``

``--parallel``
  colcon uses the parallel execution by default.
  To build packages sequentially use ``--executor sequential``.

ament build
-----------

``colcon build ...``

``--cmake-args -D... --``
  ``--cmake-args " -D..."``
  Any CMake arguments which start with a dash need to be prefixed with a space.
  This can be done by quoting each argument with a leading space.
  The closing double dash is not necessary anymore.

``--force-cmake-configure``
  ``--cmake-force-configure``

``--use-ninja``
  ``--cmake-args " -G" Ninja``
  See ``--cmake-args`` for the reason for the quoting and prefixed space.

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

Behavioral changes
------------------

``--retest-until-fail`` with ``colcon`` uses `pytest-repeat <https://github.com/pytest-dev/pytest-repeat>`_ which runs individual tests of a package N times each (the first test N times, then the second test N times, etc).
With ``ament_tools`` the entire test suite of a package was run up to N times.
As a consequence ``colcon`` provides a more accurate result since each test that passed has actually run N times.

The location of JUnit test results file for ``ament_python`` packages tested with ``colcon`` is in ``<pkg-build>/pytest.xml``, whereas with ``ament_tools`` it is in ``<pkg-build>/test_results/<pkgname>/pytest.xunit.xml``.
