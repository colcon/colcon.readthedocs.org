Mixin arguments
===============

Mixins can be used by several verbs to contribute command line arguments
defined in external files.

The following arguments are provided by the ``colcon-mixin`` package:

.. _mixin-args_mixin-files_arg:

\--mixin-files [FILE [FILE ...]]
  Read additional mixin files and make the mixin names specified in them
  available in the ``--mixin`` argument.

.. _mixin-args_mixin_arg:

\--mixin [mixin1 [mixin2 ...]]
  The names of mixins to be used.
  The list of mixin names and their command line arguments depends on which
  ones are available.
  To enumerate the available verb specific mixins and their command line
  arguments invoke ``colcon mixin show <verb>``.

  An example mixin provided in the `colcon-mixin-repository
  <https://github.com/colcon/colcon-mixin-repository/>`_ repository:

  * **debug**:

    - ``cmake-args``: ``['-DCMAKE_BUILD_TYPE=Debug']``

When multiple mixins are used they are interpreted in order with later mixin
values replacing or extending previous values.
In case of lists the items are being concatenated.
In all other cases the latter value replaces the former value.

.. warning::

    At the moment the logic concatenating lists has no semantic knowledge of
    the data.
    Therefore the joined list might not have the desired semantic meaning.
    E.g. for the following two lists:

    * ``cmake-args``: ``['-DCMAKE_C_FLAGS=-fPIC']``
    * ``cmake-args``: ``['-DCMAKE_C_FLAGS=-g']``

    the joined list will be just a concatenation:

    * ``cmake-args``: ``['-DCMAKE_C_FLAGS=-fPIC', '-DCMAKE_C_FLAGS=-g']``

    But passing these arguments to CMake would result in the latter value of
    ``CMAKE_C_FLAGS`` overwriting the former even though the user likely wanted
    both compiler options to be used.

    Furthermore, the option values of a mixin used through the CLI override the default values of a :ref:`default file <configuration_defaults-yaml>`.

Mixins referencing other mixins
-------------------------------

A mixin can be composed out of other mixins by using the reserved ``mixin``
key.
Its value is a list of names of other mixins defined for the same verb.

An example mixin file defining a mixin which references two other mixins:

.. code-block:: yaml

    {
        "build": {
            "debug": {
                "cmake-args": ["-DCMAKE_BUILD_TYPE=Debug"]
            },
            "compile-commands": {
                "cmake-args": ["-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
            },
            "develop": {
                "mixin": ["debug", "compile-commands"],
                "cmake-args": ["-DCMAKE_VERBOSE_MAKEFILE=ON"]
            }
        }
    }

Invoking ``colcon build --mixin develop`` is equivalent to using the ``debug``
and the ``compile-commands`` mixin followed by the arguments defined by the
``develop`` mixin itself:

* ``cmake-args``: ``['-DCMAKE_BUILD_TYPE=Debug', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON', '-DCMAKE_VERBOSE_MAKEFILE=ON']``

The referenced mixins are always applied before the mixin referencing them.
Therefore the referencing mixin is applied last and its values replace or
extend the values of the mixins it references, following the same rules as
described above.
The ``mixin`` key itself is only used to resolve the references, it is never
passed as a command line argument.

References are resolved recursively, so a referenced mixin can reference
further mixins itself.
The resolution happens depth first, meaning for each referenced mixin all of
its own references are applied before it.
If the same mixin is reached through more than one path it is applied once for
each path to keep the semantic that the value applied last wins.

Values passed explicitly on the command line still take precedence over the
values of all mixins, independent of whether a mixin was requested on the
command line or through a reference.

.. note::

    Using a mixin which can't be resolved results in an error.
    That is the case if a referenced mixin doesn't exist, if the references
    form a cycle or if the value of a ``mixin`` key isn't a list of strings.
