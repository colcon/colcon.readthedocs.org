Mixin arguments
===============

Mixins can be used by several verbs to contribute command line arguments
defined in external files.

The following arguments are provided by the ``colcon-mixin`` package:

.. _mixin-args_mixin-files_arg:

\\--mixin-files [FILE [FILE ...]]
  Read additional mixin files and make the mixin names specified in them
  available in the ``--mixin`` argument.

.. _mixin-args_mixin_arg:

\\--mixin [mixin1 [mixin2 ...]]
  The names of mixins to be used.
  The list of mixin names and their command line arguments depends on which
  ones are available.
  To enumerate the available verb specific mixins and their command line
  arguments invoke ``colcon mixin show <verb>``.

  An example mixin provided in the `colcon-mixin-repository
  <https://github.com/colcon/colcon-mixin-repository/>`_ repository:

  * **debug**:

    - ``cmake-args``: ``['-DCMAKE_BUILD_TYPE=Debug']``

It is important to note that in their current implementation, mixins do not
possess semantic knowledge about CMake arguments that would allow them to
execute merges of **arguments that appear multiple times** correctly.  Those
conditions arise, for example, when **mixin1** and **mixin2** contain
overlapping parameters, or when any **mixin** specifies an argument that also
appears on the command line.  This situation should be avoided since one of
the parameters will be silently suppressed and the outcome not be as expected.
Case in point:

\\--mixin debug coverage-gcc
  Will *clash* since both mixins share the ``--cmake-args`` parameter.  One of
  them will be ignored in the final CMake call.

\\--cmake-args -DCMAKE_BUILD_TYPE=Debug --mixin coverage-gcc
  Will *clash* since mixin and command line share the ``--cmake-args``
  parameter.

Arguments should thus always be *mutually exclusive*.
