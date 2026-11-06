.. _so2:
  
SO(2)
=====
.. automodule:: pglfd.geom.so2


Basic Functions
---------------
.. autofunction:: rotmat2vector
.. autofunction:: vector2rotmat
.. autofunction:: normalize
.. autofunction:: multiply
.. autofunction:: inverse

Manifold Functions
------------------
.. autofunction:: log
.. autofunction:: exp
.. autofunction:: parallel_transport

C++ API
-------
.. doxygenclass:: pglfd::SO2
   :members:

Matlab API
----------
.. mat:currentmodule:: lib
.. mat:automodule:: pglfd.geom.so2
.. mat:autofunction:: pglfd.geom.so2.multiply
.. mat:autofunction:: pglfd.geom.so2.inverse

.. mat:autofunction:: pglfd.geom.so2.log
.. mat:autofunction:: pglfd.geom.so2.exp
.. mat:autofunction:: pglfd.geom.so2.parallel_transport