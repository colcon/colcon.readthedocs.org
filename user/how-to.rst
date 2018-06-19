How to
======

This section describe how to perform common tasks.

Show all output immediately on the console
------------------------------------------

.. code-block:: bash

    $ colcon <verb> --event-handler console_direct+

.. note::

    If you use the parallel executor (which is the default when that extension is installed) the output of packages processed in parallel will be interleaved.

Build only a single package (or selected packages)
--------------------------------------------------

.. code-block:: bash

    $ colcon build --packages-select <name-of-pkg>
    $ colcon build --packages-select <name-of-pkg> <name-of-another-pkg>

.. note::

    This assumes that you have built dependencies of the selected packages within the workspace before.

Build selected packages including their dependencies
----------------------------------------------------

.. code-block:: bash

    $ colcon build --packages-up-to <name-of-pkg>

Rebuild packages which depend on a specific package
---------------------------------------------------

Assuming you have build the whole workspace before and then made changes to one package.
In order to rebuild this package as well as all packages which (recursively) depend on this package invoke:

.. code-block:: bash

    $ colcon build --packages-above <name-of-pkg>

Test selected packages as well as their dependents
--------------------------------------------------

If you have build the relevant packages before you can tnu the tests the same way as described in the previous section:

.. code-block:: bash

    $ colcon test --packages-above <name-of-pkg>

If you haven't build the relevant packages before you can do that by using one invocation to determine all dependents and a second invocation to invoke the actual build:

.. code-block:: bash

    $ colcon list -n --packages-above <name-of-pkg>
    $ colcon build --packages-up-to <copy-n-paste-output-previous-command>

Run specific tests
------------------

Depending on the type of the package a different tool is being used to run tests.

Python packages using pytest
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    $ colcon test --packages-select <name-of-pkg> --pytest-args ...

Pytest provides multiple ways to select individual tests:

* Tests can be identified by their name:

  .. code-block:: bash

      $ ... --pytest-args -k name_of_the_test_function

* Tests can be identified using markers if the tests have been decorated with markers before:

  .. code-block:: bash

      $ ... --pytest-args -m marker_name

Both approaches also support logical expressions like ``or`` and ``not``.
For more information see the `pytest documentation <https://docs.pytest.org/en/latest/example/markers.html>`_.

CMake packages using CTest
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    $ colcon test --packages-select <name-of-pkg> --ctest-args ...

CTest provides multiple ways to select individual tests:

* Tests can be selected / excluded using a regular expression matching their name:

  .. code-block:: bash

      $ ... --ctest-args -R regex
      $ ... --ctest-args -E regex

* Tests can be selected / excluded using a regular expression matching their label (which have to be assigned to each test when adding the test in the CMake code):

  .. code-block:: bash

      $ ... --ctest-args -L regex
      $ ... --ctest-args -LE regex

For more information see the `CTest documentation <https://cmake.org/cmake/help/latest/manual/ctest.1.html#options>`_.

Enable additional output for debugging
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Beside the output of the actually invoked commands to build or test packages the tool by default only outputs warning or error messages.
For debugging purposes you can enable logging messages with other levels (e.g. ``info``, ``debug``).

.. code-block:: bash

    $ colcon --log-level info <verb> ...

Log files of past invocations
-----------------------------

By default the ``log`` directory is created as a sibling to the ``src`` directory.
Some verbs (e.g. ``build``, ``test``, ``test-result``) generate log files in a subdirectory which is named following the pattern ``<verb>_<timestamp>``.
For the latest invocation of a specific verb there is a symlink named ``latest_<verb>`` (on platforms which support symbolic links).
For the latest invocation there is another symlink just named ``latest`` (on platforms which support symbolic links).

Each log directory contains a couple of files in the root:

* ``events.log`` contains all internal events dispatched.
  This file is mostly for debugging purposes.
* ``logger_all.log`` contains all logging messages even though the invocation didn't show them on the console.
  This is helpful to see log message with a different level after a command was run.
  The first line of this file contains the exact command line invocation including all the arguments passed.

For each package additional files are being created in a subdirectory named after the package:

* ``command.log`` contains the commands which have been invoked for the package, e.g. calls to ``python setup.py``.
* ``stdout.log`` contains all the output the invoked commands printed to ``stdout``.
* ``stderr.log`` contains all the output the invoked commands printed to ``stderr``.
* ``stdout_stderr.log`` contains all the output the invoked commands printed to either of the two pipes in the order they appeared.
* ``streams.log`` combines the output of all the other log files in the order they appeared.

.. note::

    While ``colcon`` is doing its best to read concurrently from the ``stdout`` and ``stderr`` pipes to preserve the order of output it can't guarantee the correctness of the order in all cases.
