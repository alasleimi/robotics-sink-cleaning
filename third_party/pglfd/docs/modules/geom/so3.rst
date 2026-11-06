.. _so3:
  
SO(3)
=====
.. automodule:: pglfd.geom.so3

Quaternion Operations
---------------------
.. autofunction:: quaternion_identity
.. autofunction:: quaternion_normalize
.. autofunction:: quaternion_inverse
.. autofunction:: quaternion_multiply
.. autofunction:: quaternion_vector_product

Conversions
-----------
.. autofunction:: quaternion_to_rotmat

.. autofunction:: axis_angle_to_rotmat
.. autofunction:: axis_angle_to_quaternion


Manifold Functions
------------------
.. autofunction:: log
.. autofunction:: exp
.. autofunction:: parallel_transport

C++ API
-------
.. doxygenclass:: pglfd::SO3
   :members:

Matlab API
----------
.. mat:currentmodule:: lib
.. mat:automodule:: pglfd.geom.so3
.. mat:autofunction:: pglfd.geom.so3.quaternion_fix
.. mat:autofunction:: pglfd.geom.so3.quaternion_inverse
.. mat:autofunction:: pglfd.geom.so3.quaternion_multiply
.. mat:autofunction:: pglfd.geom.so3.quaternion_vector_multiply

.. mat:autofunction:: pglfd.geom.so3.quaternion_to_rotmat
.. mat:autofunction:: pglfd.geom.so3.rotmat_to_quaternion

.. mat:autofunction:: pglfd.geom.so3.axis_angle_to_quaternion
.. mat:autofunction:: pglfd.geom.so3.axis_angle_to_rotmat

.. mat:autofunction:: pglfd.geom.so3.log
.. mat:autofunction:: pglfd.geom.so3.exp
.. mat:autofunction:: pglfd.geom.so3.parallel_transport
