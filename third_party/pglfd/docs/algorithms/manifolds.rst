.. _examples_manifolds:
  
Riemannian Manifolds
====================

Manifold Basics
---------------

.. _examples_manifolds_s2_tangent:
.. figure:: images/manifold/s2_tangent.*
   :alt: :math:`\mathcal{S}^2` manifold and tangent space.
   :width: 50%
   :align: center

   :math:`\mathcal{S}^2` manifold and tangent space.

A :math:`m`-dimensional differentiable manifold :math:`\mathcal{M}` such as the :math:`2`-sphere depicted above is a topological space that is locally diffeomorphic to the Euclidean space :math:`\mathbb{R}^m`.
The tangent space to :math:`\mathcal{M}` at :math:`\boldsymbol{x}_1` is denoted as :math:`\mathcal{T}_{\boldsymbol{x}_1}\mathcal{M}` and has vector space properties.
The logarithm :math:`\boldsymbol{u}_{12} = \mathrm{Log}_{\boldsymbol{x}_1}^\mathcal{M} \boldsymbol{x}_2` maps :math:`\boldsymbol{x}_2` into the tangent space of :math:`\boldsymbol{x}_1`,
with the exponential map :math:`\boldsymbol{x}_2 = \mathrm{Exp}_{\boldsymbol{x}_1}^\mathcal{M} \boldsymbol{u}_{12}` locally performing the inverse, mapping :math:`\boldsymbol{u}_{12}` back to the manifold.

Using the standard inner product, we can measure the geodesic distance according to the manifold metric between :math:`\boldsymbol{x}_1` and :math:`\boldsymbol{x}_2` using :math:`|| \boldsymbol{u}_{12} ||`.
A weighted Mahalanobis distance :math:`d_{\boldsymbol{A}}^{\mathcal{M}} (\boldsymbol{x}_1, \boldsymbol{x}_2)` with weighting matrix :math:`\boldsymbol{A} \in \mathcal{T}_{\boldsymbol{x}_1}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_1}\mathcal{M}` can be calculated to

.. math::
   :label: examples_manifolds_weighted_distance

   d_{\boldsymbol{A}}^{\mathcal{M}} (\boldsymbol{x}_1, \boldsymbol{x}_2) = \sqrt{\boldsymbol{u}_{12}^\top \boldsymbol{A} \boldsymbol{u}_{12}}.

With the :ref:`examples_gaussian`, the covariance matrix is used as weighting matrix.
Similarly, the potential function for our Riemannian :ref:`examples_control_impedance` uses the stiffness :math:`\boldsymbol{K} \in \mathcal{T}_{\boldsymbol{x}_\mathrm{ee}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_\mathrm{ee}}\mathcal{M}` as weighting matrix.

We define an action function for all manifolds - on manifolds which are also Lie groups, this is the group action and on :math:`\mathcal{S}^2` the 3D rotations of :math:`\mathrm{SO}(3)` on the standard embedding of :math:`\mathcal{S}^2` :cite:p:`zeestraten2017manifold`, see also: :meth:`pglfd.manif.s2.S2Manifold.action`.
This action function is also used to define tangent bases for all tangent spaces by translating a basis at an arbitrarily defined origin (called ``id_elem``) to the respective tangent space.
For moving vectors between different tangent spaces, parallel transport is employed.

These operations are implemented in Python, C++ and Matlab, see :ref:`manifold`.


Orientations
^^^^^^^^^^^^

Orientations are commonly represented as rotation matrices :math:`\boldsymbol{R}_{3 \times 3} \in \mathrm{SO}(3)` embedded in :math:`\mathbb{R}^9`.
Rotation matrices are orthogonal (:math:`\boldsymbol{R}^\top \boldsymbol{R} = \boldsymbol{I}`); we use right-handed matrices (:math:`\mathrm{det}(\boldsymbol{R}) = 1`).

The tangent space at the origin :math:`\boldsymbol{x}_1 = \boldsymbol{I}` is represented by rotation vectors :math:`\boldsymbol{\theta} = \theta \boldsymbol{e}` where :math:`\theta` is the rotation angle and :math:`\boldsymbol{e}` the axis.
Tangent spaces at other points can be identified using the group action, the rotation angle :math:`\theta` serves as Riemannian distance measure.

The smallest singularity-free representation of orientation are unit quaternions :math:`\boldsymbol{q} \in \mathcal{S}^2`.
Quaternions double-cover :math:`\mathrm{SO}(3)`: :math:`\boldsymbol{q}` and :math:`-\boldsymbol{q}` denote the same orientation.
For computational reasons, `pglfd` uses quaternions to represent orientations, avoiding the double cover by wrapping the logarithm at a full rotation (:math:`\mathrm{Log}_{\boldsymbol{q}} \left( -\boldsymbol{q} \right) = \boldsymbol{0}`); conversion functionality is provided in :ref:`so3`.

In Python, the :math:`\mathrm{SO}(3)` manifold can be utilized as follows:

.. literalinclude:: ../../python/examples/manifolds/so3.py
   :lines: 23-


Poses
^^^^^

Poses are commonly represented as homogeneous transformation matrices

.. math::
   :label: examples_poses_trafomat

   \boldsymbol{T}_{4 \times 4} = \begin{bmatrix} \boldsymbol{R} & \boldsymbol{p}\\ \boldsymbol{0} & 1 \end{bmatrix} \in \mathrm{SE}(3).

For convenience and storage reasons, they are stored as 3-dimensional position vector :math:`\boldsymbol{p}` and quaternion :math:`\boldsymbol{q}` in `pglfd`, i.e.

.. math::
     :label: examples_poses_pose

     \boldsymbol{\xi} = \left[ \boldsymbol{p} \quad \boldsymbol{q} \right]^\top.

Functions to convert between both representations are provided in :ref:`se3`.

While :math:`\mathrm{SE}(3)` is a Lie group, there is not bi-invariant metric for its Lie algebra.
The screw motions parameterized by the :math:`Log` and :math:`Exp` functions of the Lie group thus do not follow a Riemannian metric.
An intuitive explanation of why screw motions may not be the shortest path can be found in :cite:p:`{Chapter 2.4}zeestraten2018programmingphd`.

.. todo:: Maybe add illustration of this non-shortest path?

Instead, the product manifold :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` can be used.
Note that special care needs to be taken when composing transformations as e.g. required for :ref:`examples_tp`.

In Python, the product manifold :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` can be used as follows:

.. literalinclude:: ../../python/examples/manifolds/se3.py
   :lines: 23-


.. _examples_manifolds_product:
Product Manifolds
^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 33 33 33
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/manifold/cartesian.png
          :alt: Cartesian manifold.

          Cartesian coordinates.
     - .. figure:: images/manifold/cylindrical.png
          :alt: Cylindrical manifold.

          Cylindrical coordinates.
     - .. figure:: images/manifold/spherical.png
          :alt: Spherical manifold.

          Spherical coordinates.

While poses can be represented as the product manifold :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` (denoted as Cartesian coordinates in the above figure), axis or spherical symmetries can be leveraged as well :cite:p:`ti2023geometric`, :cite:p:`muehlbauer2026unified`.
`pglfd` provides functionality to transform from and to :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` in :ref:`conversions`.


In Python, these conversions can be used as follows:

.. literalinclude:: ../../python/examples/manifolds/conversions.py
   :lines: 23-

The same conversions are also available in C++ and Matlab, the Simulink :ref:`Controllers <pgctrl>` also implement those product manifolds, providing impedance and velocity control on those geometries.


Tangent and Cotangent Vectors
-----------------------------

The tangent space :math:`\mathcal{T}_{\boldsymbol{x}}\mathcal{M}` is the :math:`m`-dimensional vector space of all velocities passing through :math:`\boldsymbol{x}`.
In case of the product manifold :math:`\mathbb{R}^3 \times \mathrm{SO}(3)`, we obtain

.. math::
   \dot{\boldsymbol{x}} = \left[ \dot{x}, \dot{y}, \dot{z}, \omega_x, \omega_y, \omega_z \right]^\top.

Tangent space elements are once contravariant and thus tensors of type :math:`(0, 1)`.
They thus transform contravariantly between manifolds with the :ref:`Jacobian <jacobian>`.
In Python, we can for example convert a velocity in :math:`\mathcal{T}_{\boldsymbol{x}}\mathbb{R}^3 \times \mathrm{SO}(3)` to :math:`\mathcal{T}_{\boldsymbol{x}}\mathrm{SO}(2) \times \mathbb{R}^2 \times \mathrm{SO}(3)` as follows:

.. literalinclude:: ../../python/examples/manifolds/jacobians_contravariant.py
   :lines: 23-


The cotangent space :math:`\mathcal{T}_{\boldsymbol{x}}^*\mathcal{M}` is the :math:`m`-dimensional vector space of all linear functionals :math:`\eta: \mathcal{T}_{\boldsymbol{x}}\mathcal{M} \rightarrow \mathbb{R}`.
In case of the product manifold :math:`\mathbb{R}^3 \times \mathrm{SO}(3)`, we for example obtain the wrench

.. math::
   \boldsymbol{w} = \left[ f_x, f_y, f_z, \tau_x, \tau_y, \tau_z \right]^\top.

Cotangent space elements are once covariant and thus tensors of type :math:`(1, 0)`.
They thus transform covariantly between manifolds with the :ref:`Jacobian <jacobian>`.
In Python, we can for example convert a wrench in :math:`\mathcal{T}_{\boldsymbol{x}}\mathrm{SO}(2) \times \mathbb{R}^2 \times \mathrm{SO}(3)` to :math:`\mathcal{T}_{\boldsymbol{x}}\mathbb{R}^3 \times \mathrm{SO}(3)` as follows:

.. literalinclude:: ../../python/examples/manifolds/jacobians_covariant.py
   :lines: 23-

These conversions can be derived from the conservation of power, :math:`\boldsymbol{w}^\top \dot{\boldsymbol{x}} = const`.
The robotic quantities :math:`\boldsymbol{M}`, :math:`\boldsymbol{K}`, :math:`\boldsymbol{D}` are twice covariant (type :math:`(0, 2)`) and their inverses are twice contravariant (type: :math:`(2, 0)`).
A mass matrix can thus, e.g., be transformed from Cartesian to cylindrical coordinates using

.. math::
   \boldsymbol{M}_\mathrm{cyl} = \left( \boldsymbol{J} \boldsymbol{M}_\mathrm{cart}^{-1} \boldsymbol{J}^\top \right)^{-1}

with :math:`\boldsymbol{J} = \frac{\delta \boldsymbol{x}_\mathrm{cyl}}{\delta \boldsymbol{x}_\mathrm{cart}}` as above.
This relation is implemented in Simulink code for the :ref:`mass transformation <pgtrafo_utilities>` often required in controllers.


Example Code
------------

.. todo:: maybe remove this example code? not really helpful


Plot S2 and its tangent space: ``examples/manifolds/plot_s2_tangent.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Code used to generate the figure on :ref:`Manifolds and Tangent Spaces <examples_manifolds_s2_tangent>`.

.. literalinclude:: ../../python/examples/manifolds/plot_s2_tangent.py
   :lines: 23-


Plot Coordinate Systems: ``examples/manifolds/plot_productmanifolds.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Code used to generate the figure on :ref:`examples_manifolds_product`.

.. literalinclude:: ../../python/examples/manifolds/plot_productmanifolds.py
   :lines: 23-