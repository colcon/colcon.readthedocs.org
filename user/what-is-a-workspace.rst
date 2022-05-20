What is a Workspace?
====================

Colcon is a command line tool to build and test multiple software packages.
Those packages are built and tested in a workspace, but what is a workspace?

What's in a workspace?
----------------------

A workspace has these parts:

* Software packages to be built and tested
* Build artifacts of all packages that have been built
* Logs of the build process
* Install artifacts of all packages that have been built

All these parts are usually put into a single folder.
Lets create a single folder for our workspace.

.. code-block:: bash

	mkdir ws

Go into the root of our new workspace.

.. code-block:: bash

	cd ws


Software packages
*****************

A workspace needs to have the source code of all the software packages to be built.
Colcon will search all subfolders of the workspace to look for packages, but an established convention is to put all the packages into a directory called ``src``.
Let's create a directory for source code.

.. code-block:: bash

	mkdir src

We'll need at least one software package in the workspace.
Let's create a Python package.

..code-block:: bash

	mkdir src/foo
	touch src/foo/setup.py
	touch src/foo/foo.py


Put this content into ``src/foo/setup.py``:

.. code-block:: Python

	from setuptools import setup

	setup()

Put this content into ``src/foo/setup.cfg``:

.. code-block:: Python
	[metadata]
	name = foo
	version = 0.1.0

	[options]
	py_modules =
	  foo
	zip_safe = true

	[options.extras_require]
	test =
	  pytest

	[tool:pytest]
	junit_suite_name = foo

Put this content into ``src/foo/foo.py``:

.. code-block:: Python

	def foo_func():
		print('Hello from foo.py')
		return True


Build artifacts
***************

When software is built, the build process often produces intermediate build artifacts.
They are usually not used directly, but they can be reused to speed up building if packages are built again.
Colcon always directs packages to build out-of-source, meaning the build artifacts are put into a directory separate from the source code.
Every package gets its own build directory, but all build directories are put into a single base directory.
That single directory is called the ``build base``, and by default it's named  ``build`` at the root of the workspace.

.. note::

	You can change where build artifacts are put using the ``--build-base`` option to ``colcon build``.

Lets build the software and see its build artifacts.

.. code-block:: bash

	# Make sure you run this command from the root of the workspace!
	# cd ws
	colcon build

You'll see these new folders: ``build``, ``install``, and ``log``.


.. code-block::

	ws
	├── build
	│	├── COLCON_IGNORE
	│	└── foo/...
	├── install/...
	├── log/...
	└── src
	    └── foo
	        ├── foo.py
	        └── setup.py

Notice the ``build`` directory has a subdirectory ``foo`` and a file ``COLCON_IGNORE``.
The ``foo`` subdirectory has all the build artifacts produced when building ``foo``.
The ``COLCON_IGNORE`` file tells colcon there are no software packages in this folder.

Logs
****

If you've built software before you know there can be a lot of console output, but you might have noticed not much was output when you ran ``colcon build``.
This output was instead written to the ``log`` directory.
Let's look at it.

.. code-block::

	log
	├── build_2022-05-20_11-50-03
	│	├── events.log
	│	├── foo
	│	│	├── command.log
	│	│	├── stderr.log
	│	│	├── stdout.log
	│	│	├── stdout_stderr.log
	│	│	└── streams.log
	│	└── logger_all.log
	├── COLCON_IGNORE
	├── latest -> latest_build
	└── latest_build -> build_2022-05-20_11-50-03

.. note::

	The ``--event-handlers`` argument can be used to output build logs to the console. For example, ``colcon build --event-handlers console_direct+`` will output everything in real time.


The directory ``log/build_<date and time>`` contains all logs from the invocation of ``colcon build``.
A new folder is created every time ``colcon build`` is run.
The symlink ``latest_build`` always point to the most recent build.

..
	TODO(sloretz) what is events.log and logger_all.log?

The folder ``log/build_<date anmd time>/foo`` contains all logs from building ``foo``.
The file ``command.log`` shows the commands colcon ran to build the package.
The files ``stderr.log`` and ``stdout.log`` show the console output produced while building ``foo``.
``stdout_stderror.log``

..
	TODO(sloretz) what is streams.log?


The ``log`` directory contains logs from building and testing packages.
We've only built ``foo``, so there are only build logs.
Let's add tests to ``foo`` and see the output.

Make a new file for the test.

.. code-block:: bash

	touch src/foo/test_foo.py

Put the following content into ``test_foo.py``:

.. code-block:: Python

	import foo

	def test_foo():
	    assert foo.foo_func()


Tell ``colcon`` to run the tests.

.. code-block:: bash

	# Make sure you run this command from the root of the workspace!
	colcon test

Lets look in the ``log`` folder again.

.. code-block::

	log
	├── build_2022-05-20_11-50-03/...
	├── COLCON_IGNORE
	├── latest -> latest_test
	├── latest_build -> build_2022-05-20_11-50-03
	├── latest_test -> test_2022-05-20_11-50-05
	└── test_2022-05-20_11-50-05
	    ├── events.log
	    ├── foo
	    │	├── command.log
	    │	├── stderr.log
	    │	├── stdout.log
	    │	├── stdout_stderr.log
	    │	└── streams.log
	    └── logger_all.log


A new symlink ``latest_test`` was created, and it points to a new folder ``log/test_<date and time>``.
This holds the console output from running the test.
Checkout ``stdout_stderr.log``  and see the output of the latest test

.. code-block:: bash

	cat log/latest_test/foo/stdout_stderr.log

.. note::

	The command ``colcon test-result`` outputs a summary of test results to the console.
	Full test output can be printed to the console in real time with
	``colcon test --event-handlers console_direct+``.


Install artifacts
*****************

Just the install space is still called a workspace.
Merged vs isolated

Look at install folder
Source shell scripts


Using multiple workspaces
-------------------------

Underlays and overlays
Warning about overriding, and link to overriding packages issues.