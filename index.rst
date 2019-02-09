colcon - collective construction
================================

`colcon`_ is a command line tool to improve the workflow of building, testing and using multiple software packages.
It automates the process, handles the ordering and sets up the environment to use the packages.

.. _colcon: http://colcon.readthedocs.io

The code is open source, and `available on GitHub`_.

.. _available on GitHub: http://github.com/colcon

The documentation exists in two version:

* `released <http://colcon.readthedocs.io/en/released/>`_: matching the latest released version of all packages
* `latest <http://colcon.readthedocs.io/en/latest/>`_: matching the latest state on the default branch of all packages

The documentation is organized into a few sections:

* :ref:`user-docs`
* :ref:`migration-docs`

Information about development is also available:

* :ref:`developer-docs`

.. _user-docs:

.. toctree::
   :maxdepth: 2
   :caption: User Documentation

   user/installation
   user/quick-start
   user/configuration
   user/how-to

.. _developer-docs:

.. toctree::
   :maxdepth: 2
   :caption: Developer Documentation

   developer/design
   developer/bootstrap
   developer/contribution
   developer/changelog

.. _migration-docs:

.. toctree::
   :maxdepth: 2
   :caption: Migrate from other build tools

   migration/ament_tools
   migration/catkin_make_isolated
   migration/catkin_tools
