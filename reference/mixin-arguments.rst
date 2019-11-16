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
