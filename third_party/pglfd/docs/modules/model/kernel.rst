.. _kernel:
  
Kernels
=======

Python API
----------
.. automodule:: pglfd.model.kernel

.. autoclass:: Kernel
   :members:

   .. automethod:: __call__

.. autoclass:: RBFKernel
   :members:

   .. automethod:: __call__

.. autoclass:: MaternKernel
   :members:

   .. automethod:: __call__

.. autoclass:: RBFKernelDiagonal
   :members:

   .. automethod:: __call__

.. autoclass:: MaternKernelDiagonal
   :members:

   .. automethod:: __call__

C++ API
-------
.. doxygenclass:: pglfd::Kernel
   :members:

.. doxygenclass:: pglfd::RBFKernel
   :members:

.. doxygenclass:: pglfd::MaternKernel
   :members:

.. doxygenclass:: pglfd::MatrixKernel
   :members:

.. doxygenclass:: pglfd::KernelProductIdentity
   :members:

.. doxygenclass:: pglfd::KernelProductDiagonalRBF
   :members:

.. doxygenclass:: pglfd::KernelProductDiagonalMatern
   :members:
