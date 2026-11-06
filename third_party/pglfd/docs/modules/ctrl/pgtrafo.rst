.. _pgtrafo:
  
Simulink Transformations
========================
.. figure:: images/pgtrafo.*
  :width: 600
  :alt: pgtrafo Simulink library

  Simulink transformations library overview.

These transformations are part of the ``pgtrafo`` block diagram.


.. raw:: latex

   \clearpage


Quaternions
-----------

Basic Operations
^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 25 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_fixQuaternion.*
          :alt: Fix quaternion.
          :width: 100%
     - .. figure:: images/pgtrafo_invertQuaternion.*
          :alt: Invert quaternion.
          :width: 100%
     - .. figure:: images/pgtrafo_multiplyQuaternion_Vector.*
          :alt: Quaternion-vector multiplication.
          :width: 100%
     - .. figure:: images/pgtrafo_multiplyQuaternions.*
          :alt: Quaternion multiplication
          :width: 100%

These basic functions fix (:mat:meth:`pglfd.geom.so3.quaternion_fix`) and invert quaternions (:mat:meth:`pglfd.geom.so3.quaternion_inverse`).
Functions to perform a quaternion-quaternion mutliplication (rotation composition, :mat:meth:`pglfd.geom.so3.quaternion_multiply`) and a quaternion-vector mutliplication (rotation of a vector, :mat:meth:`pglfd.geom.so3.quaternion_vector_multiply`) are also provided.


Quaternion Conversions
^^^^^^^^^^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_quaternionToRotmat.*
          :alt: Quaternion to rotation matrix.
          :width: 100%
     - .. figure:: images/pgtrafo_rotmatToQuaternion.*
          :alt: Rotation matrix to quaternion.
          :width: 100%

These blocks wrap a conversion from a quaternion to a rotation matrix (:mat:meth:`pglfd.geom.so3.quaternion_to_rotmat`) and the inverse (:mat:meth:`pglfd.geom.so3.rotmat_to_quaternion`).


.. raw:: latex

   \clearpage

Poses
-----

A pose in ``pglfd`` is represented by a 7-d vector: :math:`\boldsymbol{x} = \left[ x, y, z, q_x, q_y, q_z, q_w \right]^\top`.


Basic Operations
^^^^^^^^^^^^^^^^
Identity Pose
^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_identityPose.*
          :alt: Fix quaternion.
          :width: 100%
     - .. figure:: images/pgtrafo_fixPose.*
          :alt: Invert quaternion.
          :width: 100%
     - .. figure:: images/pgtrafo_invertPose.*
          :alt: Quaternion-vector multiplication.
          :width: 100%
     - .. figure:: images/pgtrafo_multiplyPoses.*
          :alt: Quaternion multiplication
          :width: 100%

These blocks provide the identity pose, i.e. the identity transformation, a block to renormalize the quaternion part in a pose, the inversion of a transformation (:mat:meth:`pglfd.geom.se3.pose_inversion`) and a composition of transformations (:mat:meth:`pglfd.geom.se3.pose_multiply`).


Extract Components
^^^^^^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_extractPosition.*
          :alt: Extract position part.
          :width: 100%
     - .. figure:: images/pgtrafo_extractQuaternion.*
          :alt: Extract quaternion part.
          :width: 100%

These blocks allow to extract the position or the quaternion part of a full pose.

Pose Conversions
^^^^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_poseToTf.*
          :alt: Pose to transformation matrix.
          :width: 100%
     - .. figure:: images/pgtrafo_tfToPose.*
          :alt: Transformation matrix to pose.
          :width: 100%

These blocks transform a 7-d pose to a :math:`4 \times 4` transformation matrix (:mat:meth:`pglfd.geom.se3.pose_to_tf`) and vice versa (:mat:meth:`pglfd.geom.se3.tf_to_pose`).


.. raw:: latex

   \clearpage

.. _pgtrafo_filters:
Filters
-------

Lowpass Filter
^^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_LowpassFilter.*
          :alt: Lowpass filter.
          :width: 100%

          Lowpass filter.
     - .. figure:: images/pgtrafo_LowpassFilterExtF_c.*
          :alt: Lowpass filter with external f_c.
          :width: 100%

          Lowpass filter with external :math:`f_c`.


These blocks filter the input signal :math:`\boldsymbol{x}` channel-wise with cutoff frequency :math:`f_c` specified in the settings dialog or as external input.
For each time step :math:`k`, they implement the equation

.. math::

   \boldsymbol{x}_{\mathrm{filt},k} = \boldsymbol{x}_{\mathrm{filt},k-1} + \alpha \left(\boldsymbol{x}_k - \boldsymbol{x}_{\mathrm{filt},k-1}\right)

with

.. math::

   \alpha = \frac{2 \pi T f_c}{2 \pi T f_c + 1}

where :math:`T` is the sample time of the system.


Block Inputs
~~~~~~~~~~~~

* **reset** (boolean): Reset the filter state to the current value.
* **x** (vector): Signal to filter channel-wise.
* **f_c** (double): Cutoff frequency.


Block Outputs
~~~~~~~~~~~~~

* **x_filt** (vector): Filtered signal.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-6em}

.. figure:: images/pgtrafo_LowpassFilter_mask.*
   :alt: Lowpass filter parameters.
   :figwidth: 33%
   :align: right

   Lowpass filter parameters.


.. raw:: latex

   \vspace{6em}

* **Parameters**
   * **Cutoff Frequency**: Cutoff frequency (if not specified as input).

.. raw:: latex

   \wrapfill




Lowpass Pose Filter
^^^^^^^^^^^^^^^^^^^
.. list-table::
   :widths: 25 25
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgtrafo_LowpassPoseFilter.*
          :alt: Lowpass pose filter
          :width: 100%

          Lowpass pose filter.
     - .. figure:: images/pgtrafo_LowpassPoseFilterExtF_c.*
          :alt: Lowpass pose filter with external f_c.
          :width: 100%

          Lowpass pose filter with external :math:`f_c`.

These blocks filter the input `pose` :math:`\boldsymbol{x} \in \mathrm{SE}(3)` with cutoff frequency :math:`f_c` specified in the settings dialog or as external input.
For each time step :math:`k`, they implement the equation

.. math::

   \boldsymbol{x}_{\mathrm{filt},k} = \mathrm{Exp}_{\boldsymbol{x}_{\mathrm{filt},k-1}}^{\mathbb{R}^3 \times \mathrm{SO}(3)} \left( \alpha \mathrm{Log}_{\boldsymbol{x}_{\mathrm{filt},k-1}}^{\mathbb{R}^3 \times \mathrm{SO}(3)} \left( \boldsymbol{x}_k \right) \right)

with the same :math:`\alpha` as for the channel-wise filter

.. math::

   \alpha = \frac{2 \pi T f_c}{2 \pi T f_c + 1}

where :math:`T` is the sample time of the system.
Note that filtering on :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` leads to an interpolation on the shortest path on the product manifold which is a straight line for the :math:`\mathbb{R}^3` part and the minimum rotation on :math:`\mathrm{SO}(3)` as opposed to a screw motion parameterized by :math:`\mathfrak{se}(3)`.


Block Inputs
~~~~~~~~~~~~

* **reset** (boolean): Reset the filter state to the current value.
* **pose** (pose): The pose :math:`\boldsymbol{x}_k \in \mathrm{SE}(3)` to filter.
* **f_c** (double): Cutoff frequency.


Block Outputs
~~~~~~~~~~~~~

* **pose_filt** (pose): Filtered pose :math:`\boldsymbol{x}_{\mathrm{filt},k} \in \mathrm{SE}(3)`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-6em}

.. figure:: images/pgtrafo_LowpassPoseFilter_mask.*
   :alt: Lowpass filter parameters.
   :figwidth: 33%
   :align: right

   Lowpass filter parameters.


.. raw:: latex

   \vspace{6em}

* **Parameters**
   * **Cutoff Frequency**: Cutoff frequency (if not specified as input).

.. raw:: latex

   \wrapfill


.. _pgtrafo_utilities:

Utilities
---------

Mass Transformation
^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgtrafo_massTransformation.*
  :width: 200
  :alt: Mass transformation

  Transform a mass matrix.

Transforms a mass matrix from one coordinate chart to another using the Jacobian.
This block implements the equation

.. math::

   \boldsymbol{M}_{\mathrm{out}} = (\boldsymbol{J} \boldsymbol{M}_{\mathrm{in}}^{-1} \boldsymbol{J}^\top + \epsilon \cdot \boldsymbol{I})^{-1}


Block Inputs
~~~~~~~~~~~~

* **Jin** (matrix): Jacobian :math:`\boldsymbol{J}` transforming velocities from input to output coordinates.
* **eps** (double): Regularization value :math:`\epsilon` for close to singular matrices.
* **Min** (matrix): Mass matrix in input coordinates.


Block Outputs
~~~~~~~~~~~~~

* **Mout** (matrix): Transformed mass matrix.


.. _pgtrafo_util_pose_to_velocity:
Pose To Velocity
^^^^^^^^^^^^^^^^
.. figure:: images/pgtrafo_poseToVelocity.*
  :width: 200
  :alt: Pose to velocity

  Pose derivative to obtain a velocity.

This block calculates the velocity from the pose difference at two time steps.
With sample time :math:`T`, the velocity is calculated through the relation

.. math::

   \dot{\boldsymbol{x}}_k = \frac{-\mathrm{Log}_{\boldsymbol{x}_k} \boldsymbol{x}_{k-1}}{T}

and afterwards :ref:`aligned with the toolframe <pgctrl_alignment>`.

Block Inputs
~~~~~~~~~~~~

* **pose** (pose): Pose :math:`\boldsymbol{x}_k` to calculate the derivative of.
* **reset** (boolean): Reset the internal state.


Block Outputs
~~~~~~~~~~~~~

* **velocity** (vector): The velocity :math:`\dot{\boldsymbol{x}}_k` in *toolframe*.
