.. _manifold:
  
Basic Operations
================

Python API
----------
.. automodule:: pglfd.manif.manifold

.. autoclass:: Manifold
  :members:

.. automodule:: pglfd.manif.euclidean
.. autoclass:: EuclideanManifold
  :members:

.. automodule:: pglfd.manif.so2
.. autoclass:: SO2Manifold
  :members:

.. automodule:: pglfd.manif.s2
.. autoclass:: S2Manifold
  :members:

.. automodule:: pglfd.manif.so3
.. autoclass:: SO3Manifold
  :members:

.. automodule:: pglfd.manif.product
.. autoclass:: ProductManifold
  :members:

C++ API
-------
.. doxygenclass:: pglfd::Manifold
   :members:

.. doxygenclass:: pglfd::EuclideanManifold
   :members:

.. doxygenclass:: pglfd::SO2Manifold
   :members:

.. doxygenclass:: pglfd::S2Manifold
   :members:

.. doxygenclass:: pglfd::SO3Manifold
   :members:

.. doxygenclass:: pglfd::ProductManifold
   :members:

.. doxygentypedef:: pglfd::SE3Manifold
.. doxygentypedef:: pglfd::CylindricalManifold
.. doxygentypedef:: pglfd::SphericalManifold

Matlab API
----------
.. mat:automodule:: pglfd.manif.manifold
.. mat:autoclass:: pglfd.manif.manifold.Manifold
    :show-inheritance:
    :members:

.. mat:autoclass:: pglfd.manif.manifold.EuclideanManifold
    :show-inheritance:
    :members:

.. mat:autoclass:: pglfd.manif.manifold.SO2Manifold
    :show-inheritance:
    :members:

.. mat:autoclass:: pglfd.manif.manifold.S2Manifold
    :show-inheritance:
    :members:

.. mat:autoclass:: pglfd.manif.manifold.SO3Manifold
    :show-inheritance:
    :members:

.. mat:autoclass:: pglfd.manif.manifold.ProductManifold
    :show-inheritance:
    :members:

.. mat:currentmodule:: pglfd.manif.util
.. mat:autofunction:: get_productmanifold
.. mat:autofunction:: get_manifold_dim_id