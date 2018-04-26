catkin_tools
============

The following describes the mapping of some ``catkin_tools`` options and arguments to the ``colcon`` command line interface.

catkin build
------------

``[PKGNAME1 ... PKGNAMEn]``
  ``--packages-up-to PKGNAME1 ... PKGNAMEn``

``--no-deps``
  ``--package-whitelist PKGNAME1 ... PKGNAMEn``

``--start-with PKGNAME``
  ``--package-start PKGNAME``

``--force-cmake``
  ``--cmake-force-configure``

``--cmake-args ... --``
  ``--cmake-args " -D..."``
  Any CMake arguments which start with a dash need to be prefixed with a space.
  This can be done by quoting each argument with a leading space.
  The closing double dash is not necessary anymore.

``-v``, ``--verbose``
  ``--event-handler console_cohesion+``

``-i``, ``--interleave-output``
  ``--event-handler console_direct+``

``--no-status``
  ``--event-handler status-``

``--no-summarize``, ``--no-summary``
  ``--event-handler summary-``

``--no-notify``
  ``--event-handler desktop_notification-``
