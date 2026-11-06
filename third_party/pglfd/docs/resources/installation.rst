.. _installation:

Installation
============

Dependencies
------------

For creating the documentation, ``doxygen`` is required to be installed; other dependencies are automatically installed in a virtual environment using ``pip``.
To build and use both C++ and Python code, we recommend installing the following packages under Ubuntu 24.04:

.. code-block:: bash

   sudo apt install doxygen python3-numpy libeigen3-dev pybind11-dev

For executing unit tests, the following additional packages are required:

.. code-block:: bash

   sudo apt install python3-scipy


Creating the Documentation
--------------------------

The documentation (html / pdf) can be created by executing

.. code-block:: bash

   make doc
   make pdf-doc


Building and Testing without Installation
-----------------------------------------

For building locally, you can use the make target ``local_install`` and source ``local_env.sh`` to obtain a local environment:

.. code-block:: bash

   make local_install
   source local_env.sh

Afterwards, you can import ``pglfd`` modules with C++ support.
Formatting, linting and unit tests can be executed using

.. code-block:: bash

   make format
   make lint
   make type
   make pytest


System Installation
-------------------

TODO provide installation instructions for system installation