.. _pgmanif:
  
Simulink Manifolds
==================
.. figure:: images/pgmanif.*
  :width: 600
  :alt: pgmanif Simulink library

  Simulink manifold library overview.

These operations are part of the ``pgmanif`` block diagram.

.. raw:: latex

   \clearpage


Basic Operations
----------------

Exponential
^^^^^^^^^^^
.. figure:: images/pgmanif_ManifoldExp.*
  :width: 200
  :alt: Manifold exponential

  Manifold exponential calculation.

Exponentiate `tangent` :math:`\boldsymbol{u}_{12}` at `base` :math:`\boldsymbol{x}_1` to obtain a new manifold element `result` :math:`\boldsymbol{x}_2`:

.. math::

   \boldsymbol{x}_2 = \mathrm{Exp}_{\boldsymbol{x}_1}^{\mathcal{M}} \boldsymbol{u}_{12}


Block Inputs
~~~~~~~~~~~~

* **tangent** (vector): Tangent vector :math:`\boldsymbol{u}_{12} \in \mathcal{T}_{\boldsymbol{x}_1} \mathcal{M}`.
* **base** (element): Manifold element :math:`\boldsymbol{x}_1 \in \mathcal{M}` to use as basis for exponentiation.


Block Outputs
~~~~~~~~~~~~~

* **result** (element): Computed manifold element :math:`\boldsymbol{x}_2 \in \mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_ManifoldExp_mask.*
   :alt: Manifold exponential parameters.
   :figwidth: 33%
   :align: right

   Manifold exponential parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to perform the operation on.

.. raw:: latex

   \clearpage


Logarithm
^^^^^^^^^
.. figure:: images/pgmanif_ManifoldLog.*
  :width: 200
  :alt: Manifold logarithm

  Manifold logarithm calculation.

Take the logarithm of `point` :math:`\boldsymbol{x}_2` at `base` :math:`\boldsymbol{x}_1` to obtain the tangent element `result` :math:`\boldsymbol{u}_{12}`:

.. math::

   \boldsymbol{u}_{12} = \mathrm{Log}_{\boldsymbol{x}_1}^{\mathcal{M}} \boldsymbol{x}_2


Block Inputs
~~~~~~~~~~~~

* **point** (element): Manifold element :math:`\boldsymbol{x}_2 \in \mathcal{M}`.
* **base** (element): Manifold element :math:`\boldsymbol{x}_1 \in \mathcal{M}` to use as basis for exponentiation.


Block Outputs
~~~~~~~~~~~~~

* **result** (vector): Computed tangent vector :math:`\boldsymbol{u}_{12} \in \mathcal{T}_{\boldsymbol{x}_1} \mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_ManifoldLog_mask.*
   :alt: Manifold logarithm parameters.
   :figwidth: 33%
   :align: right

   Manifold logarithm parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to perform the operation on.

.. raw:: latex

   \clearpage


Action
^^^^^^
.. figure:: images/pgmanif_ManifoldAction.*
  :width: 200
  :alt: Manifold action

  Manifold action calculation.

Perform a manifold action function, moving `point` :math:`\boldsymbol{x}_{\mathrm{inp}} \in \mathcal{M}` from the chart at :math:`\boldsymbol{g} \in \mathcal{M}` to the chart at :math:`\boldsymbol{h} \in \mathcal{M}` to obtain the manifold element `result` :math:`\boldsymbol{x}_{\mathrm{res}} \in \mathcal{M}`.
On manifolds with a group structure, this corresponds to the group action

.. math::

   \boldsymbol{x}_{\mathrm{res}} = \boldsymbol{h} \boldsymbol{g}^{-1} \boldsymbol{x}_{\mathrm{inp}}.

On :math:`\mathcal{S}^2`, the action function corresponds to the action of elements on :math:`\mathrm{SO}(3)` on :math:`\mathcal{S}^2` :cite:p:`zeestraten2017manifold`.
Specifically,

.. math::

   \boldsymbol{x}_{\mathrm{res}} = \boldsymbol{h}_{\mathrm{SO}(3)} \boldsymbol{g}_{\mathrm{SO}(3)}^{-1} \boldsymbol{x}_{\mathrm{inp}}

where the elements :math:`\boldsymbol{(.)}_{\mathrm{SO}(3)}` correspond to the :math:`\mathrm{SO}(3)` action rotating an arbitrarily defined origin to the element :math:`\boldsymbol{(.)} \in \mathcal{S}^2`, see also: :mat:meth:`pglfd.manif.manifold.S2Manifold.action`.


Block Inputs
~~~~~~~~~~~~

* **point** (element): Manifold element :math:`\boldsymbol{x}_{\mathrm{inp}} \in \mathcal{M}`.
* **g** (element): Manifold element :math:`\boldsymbol{g} \in \mathcal{M}` to use as origin for `point`.
* **h** (element): Manifold element :math:`\boldsymbol{h} \in \mathcal{M}` to use as origin for `result`.


Block Outputs
~~~~~~~~~~~~~

* **result** (element): Computed element :math:`\boldsymbol{x}_{\mathrm{res}} \in \mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_ManifoldAction_mask.*
   :alt: Manifold action parameters.
   :figwidth: 33%
   :align: right

   Manifold action parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to perform the operation on.

.. raw:: latex

   \clearpage


Parallel Transport
^^^^^^^^^^^^^^^^^^
.. figure:: images/pgmanif_ManifoldParallelTransport.*
  :width: 200
  :alt: Manifold parallel transport

  Parallel transport of a vector or matrix from ``g`` to ``h``.

Perform a parallel transport along the Levi-Civita connection, moving `input` :math:`\boldsymbol{u}_{\mathrm{inp}}` from the tangent space at :math:`\boldsymbol{g}` to the tangent space at :math:`\boldsymbol{h}` to obtain the tangent element `result` :math:`\boldsymbol{u}_{\mathrm{res}}`.
Specifically,

.. math::

   \boldsymbol{u}_{\mathrm{res}} = \Gamma_{\boldsymbol{h} \rightarrow \boldsymbol{g}} \boldsymbol{u}_{\mathrm{inp}}



Block Inputs
~~~~~~~~~~~~

* **input** (vector / matrix): Tangent vector :math:`\boldsymbol{u}_{\mathrm{inp}} \in \mathcal{T}_{\boldsymbol{g}} \mathcal{M}` / matrix :math:`\boldsymbol{u}_{\mathrm{inp}} \in \mathcal{T}_{\boldsymbol{g}} \mathcal{M} \times \mathcal{T}_{\boldsymbol{g}} \mathcal{M}`.
* **g** (element): Manifold element :math:`\boldsymbol{g} \in \mathcal{M}` to use as tangent space location for `input`.
* **h** (element): Manifold element :math:`\boldsymbol{h} \in \mathcal{M}` to use as tangent space location for `result`.


Block Outputs
~~~~~~~~~~~~~

* **result** (vector / matrix): Computed tangent vector :math:`\boldsymbol{u}_{\mathrm{res}} \in \mathcal{T}_{\boldsymbol{h}} \mathcal{M}` / matrix :math:`\boldsymbol{u}_{\mathrm{res}} \in \mathcal{T}_{\boldsymbol{h}} \mathcal{M} \times \mathcal{T}_{\boldsymbol{h}} \mathcal{M}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_ManifoldAction_mask.*
   :alt: Parallel transport parameters.
   :figwidth: 33%
   :align: right

   Parallel transport parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to perform the operation on.

.. raw:: latex

   \clearpage


Conversion
----------

R3*SO3 To Manifold
^^^^^^^^^^^^^^^^^^
.. figure:: images/pgmanif_R3_SO3ToManifold.*
  :width: 200
  :alt: R3*SO3 to Manifold

  Convert a 7d pose to a manifold vector.

Convert `r3*so3` :math:`\boldsymbol{x}_{\mathrm{in}}^{\mathbb{R}^3 \times \mathrm{SO}(3)}` to the manifold element `point` :math:`\boldsymbol{x}_{\mathrm{out}}^{\mathcal{M}}`.


Block Inputs
~~~~~~~~~~~~

* **r3*so3** (element): Element to convert :math:`\boldsymbol{x}_{\mathrm{in}}^{\mathbb{R}^3 \times \mathrm{SO}(3)}`.


Block Outputs
~~~~~~~~~~~~~

* **point** (element): Converted element :math:`\boldsymbol{x}_{\mathrm{out}}^{\mathcal{M}}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_R3_SO3ToManifold_mask.*
   :alt: R3*SO3 to manifold parameters.
   :figwidth: 33%
   :align: right

   :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` to :math:`\mathcal{M}` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to convert to.

.. raw:: latex

   \clearpage


Manifold To R3*SO3
^^^^^^^^^^^^^^^^^^
.. figure:: images/pgmanif_ManifoldToR3_SO3.*
  :width: 200
  :alt: Manifold to R3*SO3

  Convert a manifold element to a 7d pose.

Convert the manifold element `point` :math:`\boldsymbol{x}_{\mathrm{in}}^{\mathcal{M}}` to `r3*so3` :math:`\boldsymbol{x}_{\mathrm{out}}^{\mathbb{R}^3 \times \mathrm{SO}(3)}`.


Block Inputs
~~~~~~~~~~~~

* **point** (element): Element to convert :math:`\boldsymbol{x}_{\mathrm{in}}^{\mathcal{M}}`.


Block Outputs
~~~~~~~~~~~~~

* **r3*so3** (element): Converted element :math:`\boldsymbol{x}_{\mathrm{out}}^{\mathbb{R}^3 \times \mathrm{SO}(3)}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_R3_SO3ToManifold_mask.*
   :alt: Manifold to R3*SO3 parameters.
   :figwidth: 33%
   :align: right

   :math:`\mathcal{M}` to :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold to convert from.

.. raw:: latex

   \clearpage


Jacobian
--------

Jacobian
^^^^^^^^
.. figure:: images/pgmanif_ManifoldJacobian.*
  :width: 200
  :alt: Manifold jacobian

  Calculate the manifold Jacobian.

Obtain the manifold `jacobian` :math:`\boldsymbol{J} \in \mathbb{R}^{6 \times 6}` mapping from :math:`\mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3)` to :math:`\mathcal{T}_{\boldsymbol{x}} \mathcal{M}` at `point` :math:`\boldsymbol{x} \in \mathcal{M}`.
`eps` specifies when the `singular` condition triggers, returning an identity for small radii.


Block Inputs
~~~~~~~~~~~~

* **point** (element): Element to calculate the jacobian at :math:`\boldsymbol{x} \in \mathcal{M}`.
* **eps** (double): Singular condition trigger for manifold radius (if any).


Block Outputs
~~~~~~~~~~~~~

* **jacobian** (matrix): Jacobian mapping velocities in :math:`\mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3)` to :math:`\mathcal{T}_{\boldsymbol{x}} \mathcal{M}`.
* **singular** (boolean): Signal if the singular condition has been triggered. In that case, :math:`\boldsymbol{J} = \boldsymbol{I}`.


Mask Parameters
~~~~~~~~~~~~~~~

.. raw:: latex

   \vspace{-2em}

.. figure:: images/pgmanif_ManifoldJacobian_mask.*
   :alt: Jacobian parameters.
   :figwidth: 33%
   :align: right

   Jacobian parameters.

* **Manifold Parameters**
   * **Manifold Description**: The manifold into which this jacobian converts.

.. raw:: latex

   \clearpage


Util
----

Task Parameterization
^^^^^^^^^^^^^^^^^^^^^
.. figure:: images/pgmanif_R3_SO3TaskParameterization.*
  :width: 200
  :alt: Task parameterization

  Perform task parameterization.

Perform task parameterization on :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` by rotating the Gaussian (`mu` :math:`\boldsymbol{\mu} \in \mathbb{R}^3 \times \mathrm{SO}(3)`, `covariance` :math:`\boldsymbol{\Sigma} \in \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3)`) by :math:`\boldsymbol{A} \in \mathbb{R}^{6 \times 6}` and moving by :math:`b \in \mathbb{R}^3 \times \mathrm{SO}(3)`.


Block Inputs
~~~~~~~~~~~~

* **mu** (element): Original mean :math:`\boldsymbol{\mu} \in \mathbb{R}^3 \times \mathrm{SO}(3)`.
* **covariance** (matrix): Original covariance :math:`\boldsymbol{\Sigma} \in \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3)`.
* **A** (matrix): Rotation :math:`\boldsymbol{A} \in \mathbb{R}^{6 \times 6}` in tangent space.
* **b** (element): Transformation :math:`b \in \mathbb{R}^3 \times \mathrm{SO}(3)` on the manifold.


Block Outputs
~~~~~~~~~~~~~

* **mu** (element): Transformed mean :math:`\boldsymbol{\mu} \in \mathbb{R}^3 \times \mathrm{SO}(3)`.
* **covariance** (matrix): Transformed covariance :math:`\boldsymbol{\Sigma} \in \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3) \times \mathcal{T}_{\boldsymbol{x}} \mathbb{R}^3 \times \mathrm{SO}(3)`.