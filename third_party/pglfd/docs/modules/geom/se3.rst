.. _se3:
  

SE(3)
=====
.. automodule:: pglfd.geom.se3

SE(3) Operations
----------------
.. autofunction:: invert_transformation
.. autofunction:: multiply_poses

C++ API
-------
.. doxygenclass:: pglfd::SE3
   :members:

Matlab API
----------
.. mat:currentmodule:: lib
.. mat:automodule:: pglfd.geom.se3
.. mat:autofunction:: pglfd.geom.se3.pose_inversion
.. mat:autofunction:: pglfd.geom.se3.pose_multiply

.. mat:autofunction:: pglfd.geom.se3.pose_to_tf
.. mat:autofunction:: pglfd.geom.se3.tf_to_pose
