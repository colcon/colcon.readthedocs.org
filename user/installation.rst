Installation
============

The functionality of ``colcon`` is split over multiple Python packages.
The package ``colcon-core`` provides the command line tool ``colcon`` itself as well as few fundamental extensions.
Additional functionality is provided by separate packages, e.g. ``colcon-cmake`` adds support for packages which use `CMake <https://cmake.org/>`_.
In the following a set of common ``colcon`` packages is being installed.

The most common way of installation for users is to the Python package manager ``pip``.

Using pip on any platform
-------------------------

The following assumes that you are using a virtual environment with Python 3.5 or higher.
If you want to install the packages globally it might be necessary to invoke ``pip3`` instead of ``pip`` and require ``sudo``.

.. code-block:: bash

    $ pip install -U colcon-common-extensions

.. note::

    The package ``colcon-common-extensions`` doesn't contain any functionality itself but only depends on a set of other packages (see `setup.cfg <https://github.com/colcon/colcon-common-extensions/blob/master/setup.cfg>`_).

.. note::

    You can find a list of released packages on `PyPI <https://pypi.org/search/?q=colcon>`_ using the keyword ``colcon``.

Shell specific packages:
~~~~~~~~~~~~~~~~~~~~~~~~

Bash
^^^^

On Linux / macOS you might want to install support for ``bash`` as well as completion within that shell.
When using the previously mentioned ``colcon-common-extensions`` that happens automatically on non-Windows platforms.

.. code-block:: bash

    $ pip install -U colcon-bash colcon-argcomplete

For the completion to work you must run the following command.
For convenience you might want to add that invocation to your shell configuration, e.g. ``~/.bashrc``:

.. code-block:: bash

    $ register-python-argcomplete colcon

PowerShell
^^^^^^^^^^

On Windows (or if you are using ``PowerShell`` on your platform) you might want to install support for it.
When using the previously mentioned ``colcon-common-extensions`` that happens automatically on Windows.

.. code-block:: bash

    $ pip install -U colcon-powershell

Installing from source
----------------------

.. note::

    This approach is commonly only used by advanced users.

Commonly this is the case when you want to try or leverage new features or bug fixes which have been committed already but are not available in a released version yet.
In order to use the latest state of any of the above packages you can invoke ``pip`` with a URL of the GitHub repository:

.. code-block:: bash

    $ pip install -U git+https://github.com/colcon/colcon-common-extensions.git

Building from source
--------------------

Since this is not a common use case for users you will find the documentation in the :doc:`developer section <../developer/bootstrap>`.
