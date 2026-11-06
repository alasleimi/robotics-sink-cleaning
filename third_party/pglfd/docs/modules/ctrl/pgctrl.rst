.. _pgctrl:
  
Simulink Control
==================

.. figure:: images/pgctrl.*
  :alt: pgctrl Simulink library

  Simulink control library overview.

.. attention:: The algorithms in this library are protected by patents. Please contact the German Aerospace Center (DLR) and the Technical University of Munich (TUM) to obtain a license for commercial applications.

These blocks are part of the ``pgctrl`` block diagram.
Use them to implement impedance controllers with geometric potential or velocity controllers with on-manifold velocities :cite:p:`muehlbauer2024probabilistic`, :cite:p:`muehlbauer2026unified`, :cite:p:`muehlbauer2026stabilizing`.

.. raw:: latex

   \clearpage


.. _pgctrl_impedance:
Impedance Control
-----------------

.. _pgctrl_impedance_impedance:
Impedance
^^^^^^^^^
.. figure:: images/pgctrl_ManifoldImpedance.*
  :width: 33%
  :alt: Manifold impedance.

  On-manifold impedance control.

Passivity-based manifold impedance control - this block combines below stiffness and damping blocks for full on-manifold impedance control :cite:p:`muehlbauer2024probabilistic`, :cite:p:`muehlbauer2026unified` with passivation :cite:p:`muehlbauer2026stabilizing`.
Usage details for the impedance control blocks can be found in the algorithm section: :ref:`examples_control_impedance`.


Block Inputs
~~~~~~~~~~~~

* **reset** (boolean): Reset internal states.
* **tool in origin** (7-d pose): Tool pose expressed in a global coordinate system.
* **target in origin** (7-d pose): Attractor pose expressed in a global coordinate system.
* **vtool in tool** (6-d vector): Tool velocity expressed in *tool coordinate system* - this is what is usually obtained from a robot.
* **vtarget in origin** (6-d vector): Attractor velocity expressed in tangent space at tool (can be obtained using :ref:`pgtrafo_util_pose_to_velocity`). Note the different coordinate system compared to the tool velocity.
* **K in tool** (6x6 tangent matrix): Stiffness matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.
* **D in tool** (6x6 tangent matrix): Damping matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Block Outputs
~~~~~~~~~~~~~

* **wrench in tool** (6-d covector): Wrench in *tool coordinate sytem*.
* **passivity info**: Structure containing internal passivity states.


.. raw:: latex

   \clearpage

Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_ManifoldImpedance_mask.*
   :alt: Manifold impedance parameters.
   :figwidth: 33%
   :align: right

   Impedance control parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.
* **Passivity Parameters**
   * **Passivity Method**: Can be any of the methods :cite:p:`muehlbauer2026stabilizing` described in :ref:`examples_control_passivation`. ``None`` disables passivation, resulting in nominal impedance control.
   * **Initial Energy (Joule)**: Passivity-violating energy that can be used. Depletes until a reset is sent to the block.
   * **Usage rate of initial energy (Watt)**: Usage rate of the initial energy to avoid jumps.
   * **Allow Asymmetric Part of Stiffness Matrix**: Enable passivation of the curl part of the stiffness matrix :cite:p:`muehlbauer2026stabilizing`. If disabled, only the symmetric part of the stiffness matrix is used.
   * **Active power for asymmetric part**: Allow passivity violation of the curl by this power value.
* **Limit delta x Parameters**
   * **Maximum increase of length per second**: Limit the increase rate of the lever when using passivation via ``Limit delta x`` to avoid jumps. We recommend to instead use ``Limit delta x discrete`` which is exact in discrete time systems.

.. raw:: latex

   \clearpage


.. _pgctrl_impedance_stiffness:
Stiffness
^^^^^^^^^
.. figure:: images/pgctrl_ManifoldStiffness.*
  :width: 33%
  :alt: Manifold stiffness.

  On-manifold stiffness.

.. note:: In general, the :ref:`pgctrl_impedance_impedance` block which wraps :ref:`pgctrl_impedance_stiffness` and :ref:`pgctrl_impedance_damping` should be used.

The *Manifold Stiffness* block implements the derivative of the on-manifold potential function

.. math::
   :label: pgctrl_stiffness_potential

   V = \frac{1}{2} {\mathrm{Log}_{\boldsymbol{x}_{\mathrm{ee}}}^{\mathcal{M}}}^\top \boldsymbol{K} \mathrm{Log}_{\boldsymbol{x}_{\mathrm{ee}}}^{\mathcal{M}}.

Note the close relation to the on-manifold Mahalanobis distance :eq:`examples_manifolds_weighted_distance`.
It supports a passivation of variable stiffness using the methods described in :cite:p:`muehlbauer2026stabilizing`.



Block Inputs
~~~~~~~~~~~~

* **reset** (boolean): Reset internal states.
* **tool in origin** (7-d pose): Tool pose expressed in a global coordinate system.
* **target in origin** (7-d pose): Attractor pose expressed in a global coordinate system.
* **K in tool** (6x6 tangent matrix): Stiffness matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.
* **Damping info**: Structure containing internal damping information for passivation.


Block Outputs
~~~~~~~~~~~~~

* **wrench in tool** (6-d covector): Wrench in *tool coordinate sytem*.
* **passivity info**: Structure containing internal passivity states.

.. raw:: latex

   \clearpage


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_ManifoldStiffness_mask.*
   :alt: Manifold stiffness parameters.
   :figwidth: 33%
   :align: right

   Stiffness mask parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.
* **Passivity Parameters**
   * **Passivity Method**: Can be any of the methods :cite:p:`muehlbauer2026stabilizing` described in :ref:`examples_control_passivation`. ``None`` disables passivation, resulting in nominal impedance control.
   * **Initial Energy (Joule)**: Passivity-violating energy that can be used. Depletes until a reset is sent to the block.
   * **Usage rate of initial energy (Watt)**: Usage rate of the initial energy to avoid jumps.
* **Limit delta x Parameters**
   * **Maximum increase of length per second**: Limit the increase rate of the lever when using passivation via ``Limit delta x`` to avoid jumps. We recommend to instead use ``Limit delta x discrete`` which is exact in discrete time systems.

.. raw:: latex

   \wrapfill



Helper Blocks
~~~~~~~~~~~~~
.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/pgctrl_StiffnessAdaptation.*
          :alt: Stiffness adaptation.
          :width: 100%

          Stiffness adaptation.
     - .. figure:: images/pgctrl_PrecisionToStiffness.*
          :alt: Precision to stiffness.
          :width: 100%

          Precision to stiffness.

The *Stiffness Adaptation* block implements the scaling of nominal stiffnesses described in :cite:p:`muehlbauer2026unified`.

The *Precision to Stiffness* block allows deriving a stiffness matrix from a given precision using the algorithm :cite:p:`muehlbauer2026unified`.
Through a decomposition of the precision matrix, fully populated stiffness matrices are derived which even support coupling between translational and rotational degrees of freedom.

.. raw:: latex

   \clearpage


.. _pgctrl_impedance_damping:
Damping
^^^^^^^
.. figure:: images/pgctrl_ManifoldDamping.*
  :width: 33%
  :alt: Manifold damping.

  On-manifold damping.

.. note:: In general, the :ref:`pgctrl_impedance_impedance` block which wraps :ref:`pgctrl_impedance_stiffness` and :ref:`pgctrl_impedance_damping` should be used.

The *Manifold Damping* block implements the manifold-valued damping calculation

.. math::

   \boldsymbol{w} = \boldsymbol{D} \Delta \dot{\boldsymbol{x}} = \boldsymbol{D} \left( \dot{\boldsymbol{x}} - \dot{\boldsymbol{x}}_{\mathrm{ee}} \right)

with attractor velocity :math:`\dot{\boldsymbol{x}}` (e.g., derived from :math:`\boldsymbol{x}` using :ref:`pgtrafo_util_pose_to_velocity`) and end effector velocity :math:`\dot{\boldsymbol{x}}_{\mathrm{ee}}`.
The resulting wrench is transformed in the *tool coordinate system* and can directly be applied in the toolframe.



Block Inputs
~~~~~~~~~~~~

* **tool in origin** (7-d pose): Tool pose expressed in a global coordinate system.
* **vtool in tool** (6-d vector): Tool velocity expressed in *tool coordinate system* - this is what is usually obtained from a robot.
* **vtarget in origin** (6-d vector): Attractor velocity expressed in tangent space at tool (can be obtained using :ref:`pgtrafo_util_pose_to_velocity`). Note the different coordinate system compared to the tool velocity.
* **D in tool** (6x6 tangent matrix): Damping matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Block Outputs
~~~~~~~~~~~~~

* **wrench in tool** (6-d covector): Wrench in *tool coordinate sytem*.
* **passivity info**: Structure containing internal passivity states.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-6em}

.. figure:: images/pgctrl_ManifoldDamping_mask.*
   :alt: Manifold damping parameters.
   :figwidth: 33%
   :align: right

   Damping mask parameters.


.. raw:: latex

   \vspace{6em}

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill



Helper Blocks
~~~~~~~~~~~~~
.. figure:: images/pgctrl_DoubleDiagonalizationDampingDesign.*
  :width: 33%
  :alt: Damping Design.

  Double diagonalization damping design.

Double diagonalization :cite:p:`albuschaeffer2003cartesianimpedance` is the multi-dimensional generalization of the damping calculation

.. math::

   d = 2 \zeta \sqrt{m k},

where :math:`m` is the mass, :math:`k` the stiffness and :math:`\zeta` the damping coefficient in a one-dimensional mass-spring-damper system.
For :math:`\zeta = 0`, the system is undamped and for :math:`\zeta = 1` the system is critically damped, any higher value for :math:`\zeta` leads to overdamping.
We commonly set :math:`\zeta = 1 / \sqrt{2} \approx 0.7` to achieve the quickest settling time while limiting overshoot.



Block Inputs
""""""""""""

* **K**  (6x6 tangent matrix): Stiffness matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.
* **zeta** (6-d vector): Damping ratio, :math:`0 \le \zeta \le 1`.
* **M** (6x6 tangent matrix): Mass matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Block Outputs
"""""""""""""

* **D** (6x6 tangent matrix): Damping matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.

.. raw:: latex

   \clearpage


Velocity Control
----------------

.. figure:: images/pgctrl_ManifoldVelocityControl.*
  :width: 33%
  :alt: Manifold velocity.

  On-manifold velocity control.

This block implements on-manifold velocity control :cite:p:`muehlbauer2026unified`.
Usage details for this block can be found in the algorithm section: :ref:`examples_control_velocity`.


Block Inputs
^^^^^^^^^^^^

* **tool in origin** (7-d pose): Tool pose expressed in a global coordinate system.
* **cart vtool** (6-d vector): Tool velocity expressed in *tool coordinate system* - this is what is usually obtained from a robot.
* **man vtarget** (6-d vector): Target velocity expressed in tangent space at tool in manifold coordinates.
* **man prec** (6x6 tangent matrix): Precision matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.
* **D** (6x6 tangent matrix): Damping matrix expressed in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Block Outputs
^^^^^^^^^^^^^

* **cart precision** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)`.
* **cart vtarget** (6-d vector): Target velocity in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)`.
* **cart wrench** (6-d covector): Wrench in *tool coordinate sytem*.
* **man velocity error** (6-d vector): Velocity error in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Mask Parameters
^^^^^^^^^^^^^^^

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_ManifoldVelocityControl_mask.*
   :alt: Manifold velocity parameters.
   :figwidth: 33%
   :align: right

   Velocity control parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill



Task Parameterization
---------------------
.. figure:: images/pgctrl_SE_3_TaskParameterization.*
  :width: 33%
  :alt: SE(3) task parameterization.

  SE(3) task parameterization.

Perform SE(3) task parameterization :cite:p:`muehlbauer2026nonparametric`.


Block Inputs
^^^^^^^^^^^^

* **mu** (7-d pose): Mean in original coordinate system.
* **covariance** (6x6 tangent matrix): Covariance in original coordinate system.
* **trafo** (7-d pose): SE(3) transformation between coordinate systems.


Block Outputs
^^^^^^^^^^^^^

* **mu** (7-d pose): transformed mean.
* **covariance** (6x6 tangent matrix): Transformed covariance.


.. _pgctrl_alignment:
Vector and Matrix Alignment
---------------------------

.. _pgctrl_alignment_wrench:
On-Manifold Wrench Alignment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_On_ManifoldWrenchAlignment.*
  :width: 33%
  :alt: On-manifold wrench alignment.

  On-manifold wrench alignment.

For wrenches :math:`\boldsymbol{w} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathbb{R}^3 \times \mathrm{SO}(3)`, position directions (:math:`x`, :math:`y`, :math:`z`) are expressed in the *origin coordinate system* while orientation directions (:math:`r_x`, :math:`r_y`, :math:`r_z`) are expressed in the *tool coordinate system* owing to the :math:`\mathrm{Log}` and :math:`\mathrm{Exp}` functions.
For robot control, we require a wrench :math:`\boldsymbol{w} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathrm{SE}(3)` where all elements are expressed in the *tool coordinate system* which is achieve by this block by rotating the position degrees of freedom.



Block Inputs
~~~~~~~~~~~~

* **R (A in B)** (3x3 rotation matrix): Rotation from A to B - tool rotation in *origin coordinate system*.
* **Wrench in A** (6-d covector): Wrench :math:`\boldsymbol{w} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathbb{R}^3 \times \mathrm{SO}(3)`.


Block Outputs
~~~~~~~~~~~~~

* **Wrench in B** (6-d covector): Wrench :math:`\boldsymbol{w} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathrm{SE}(3)` (fully in *tool coordinate system*).

On-Manifold Matrix Alignment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_On_ManifoldMatrixAlignment.*
  :width: 33%
  :alt: On-manifold matrix alignment.

  On-manifold matrix alignment.

Similar to the :ref:`pgctrl_alignment_wrench`, matrices have to be aligned.



Block Inputs
~~~~~~~~~~~~

* **R (A in B)** (3x3 rotation matrix): Rotation from A to B - tool rotation in *origin coordinate system*.
* **Matrix in A** (6x6 tangent matrix): Matrix :math:`\boldsymbol{A} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathbb{R}^3 \times \mathrm{SO}(3)`.


Block Outputs
~~~~~~~~~~~~~

* **Matrix in B** (6-d covector): Wrench :math:`\boldsymbol{A} \in \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathrm{SE}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}} \mathrm{SE}(3)` (fully in *tool coordinate system*).





Precision Transformations
-------------------------

Precision from R3*SO3
^^^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_PrecisionFromR3_SO_3_.*
  :width: 33%
  :alt: Precision from R3*SO3.

  Precision from :math:`\mathbb{R}^3 \times \mathrm{SO}(3)`.

Convert precision in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)` to a manifold-valued precision :cite:p:`muehlbauer2026unified`.


Block Inputs
~~~~~~~~~~~~

* **mu** (7-d pose): Tool pose expressed in a global coordinate system.
* **R3*SO(3) precision** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)`.


Block Outputs
~~~~~~~~~~~~~

* **precision** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_PrecisionFromR3_SO_3__mask.*
   :alt: Precision parameters.
   :figwidth: 33%
   :align: right

   Precision from :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill




Precision to R3*SO3
^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_PrecisionToR3_SO_3_.*
  :width: 33%
  :alt: Precision to R3*SO3.

  Precision to :math:`\mathbb{R}^3 \times \mathrm{SO}(3)`.

Convert a manifold-valued precision matrix to :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)` :cite:p:`muehlbauer2026unified`.


Block Inputs
~~~~~~~~~~~~

* **mu** (7-d pose): Tool pose expressed in a global coordinate system.
* **precision** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Block Outputs
~~~~~~~~~~~~~

* **R3*SO(3) precision** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathbb{R}^3 \times \mathrm{SO}(3)`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_PrecisionToR3_SO_3__mask.*
   :alt: Precision parameters.
   :figwidth: 33%
   :align: right

   Precision to :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill



Precision from SE(3)
^^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_PrecisionFromSE_3_.*
  :width: 33%
  :alt: Precision from SE3.

  Precision from :math:`\mathrm{SE}(3)`.

Convert precision in tool coordinate system to a manifold-valued precision :cite:p:`muehlbauer2026unified`.


Block Inputs
~~~~~~~~~~~~

* **proxy in tool** (7-d pose): Proxy pose expressed in tool coordinate system.
* **precision in tool SE(3)** (6x6 tangent matrix): Precision matrix in tool coordinate system.
* **tool in origin** (7-d pose): Tool pose expressed in origin coordinate system.


Block Outputs
~~~~~~~~~~~~~

* **proxy in origin** (7-d pose): Proxy pose expressed in origin coordinate system.
* **precision in tool** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_PrecisionFromSE_3__mask.*
   :alt: Precision parameters.
   :figwidth: 33%
   :align: right

   Precision from :math:`\mathrm{SE}(3)` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill




Precision to SE(3)
^^^^^^^^^^^^^^^^^^
.. figure:: images/pgctrl_PrecisionToSE_3_.*
  :width: 33%
  :alt: Precision to SE3.

  Precision to :math:`\mathrm{SE}(3)`.

Convert a manifold-valued precision matrix to a precision in tool coordinate system :cite:p:`muehlbauer2026unified`.


Block Inputs
~~~~~~~~~~~~

* **proxy in origin** (7-d pose): Proxy pose expressed in origin coordinate system.
* **precision in tool** (6x6 tangent matrix): Precision matrix in :math:`\mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M} \times \mathcal{T}_{\boldsymbol{x}_{\mathrm{ee}}}\mathcal{M}`.
* **tool in origin** (7-d pose): Tool pose expressed in a global coordinate system.


Block Outputs
~~~~~~~~~~~~~

* **proxy in tool** (7-d pose): Proxy pose expressed in tool coordinate system.
* **precision in tool SE(3)** (6x6 tangent matrix): Precision matrix in tool coordinate system.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgctrl_PrecisionToSE_3__mask.*
   :alt: Precision parameters.
   :figwidth: 33%
   :align: right

   Precision to :math:`\mathrm{SE}(3)` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold underlying this controller.
   * **Epsilon for Manifold Radius**: On manifolds with *radius* degree of freedom, assert singularity if below this threshold.

.. raw:: latex

   \wrapfill