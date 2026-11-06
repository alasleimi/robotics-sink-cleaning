.. _examples_gaussian:
  
Riemannian Gaussian Distribution
================================

The Riemannian Gaussian distribution :cite:p:`zeestraten2017manifold`, :cite:p:`calinon2020gaussians` is one possible generalization of the multivariate normal distribution to Riemannian manifolds.
Its probability density function is given as

.. math::
   :label: examples_gaussian_pdf

   \mathcal{N}\left(\boldsymbol{x}\vert\boldsymbol{\mu},\boldsymbol{\Sigma}\right) = \frac{1}{\sqrt{\left(2\pi\right)^D\mathrm{det}(\boldsymbol{\Sigma})}}e^{-\frac{1}{2}\mathrm{Log}_{\boldsymbol{\mu}}^{\mathcal{M}_i}\!\left(\boldsymbol{x}\right)^\top\boldsymbol{\Sigma}^{-1}\mathrm{Log}_{\boldsymbol{\mu}}^{\mathcal{M}_i}\!\left(\boldsymbol{x}\right)},

where :math:`\boldsymbol{x}` is the test point, :math:`\boldsymbol{\mu} \in \mathcal{M}` the mean and :math:`\boldsymbol{\Sigma} \in \mathcal{T}_{\boldsymbol{\mu}} \mathcal{M} \times \mathcal{T}_{\boldsymbol{\mu}} \mathcal{M}` the covariance of the Gaussian.
The :ref:`gaussian` implementation in `pglfd` can be initialized either from mean and covariance or fitted from data as presented in the following.


Basic Operations
----------------


Fitting from Data
^^^^^^^^^^^^^^^^^

.. figure:: images/gaussian/gaussian.*
   :alt: Riemannian Gaussian on :math:`\mathcal{S}^2`.
   :width: 50%
   :align: center

   Riemannian Gaussian on :math:`\mathcal{S}^2`.

.. todo:: cite letter data, potentially remove?

A Riemannian Gaussian distribution is fitted by first calculating the :ref:`Fréchet mean <util>` (depicted as red star) of the manifold-valued data (depicted as yellow lines), followed by calculating the covariance in tangent space (depicted as red ellipse).
Details can be found in :cite:p:`zeestraten2017manifold`.
In `pglfd`, this iterative fitting is performed in the :meth:`pglfd.model.gaussian.Gaussian.mle_from_data` method as shown in the :ref:`example code <examples_gaussian_basic_example>`.


Probability of Samples
^^^^^^^^^^^^^^^^^^^^^^

The probability of a sample is calculated by :meth:`pglfd.model.gaussian.Gaussian.probability` which evaluates the probability density function :eq:`examples_gaussian_pdf`.
For some exemplary points, we get:

.. literalinclude:: images/gaussian/gaussian_probability.txt

.. _examples_gaussian_sampling:

Sampling from a Distribution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/gaussian/gaussian_sample.*
   :alt: Sampling from a Riemannian Gaussian on :math:`\mathcal{S}^2`.
   :width: 50%
   :align: center

   Sampling from a Riemannian Gaussian on :math:`\mathcal{S}^2`.

Similarly, sampling from a Riemannian Gaussian distribution is performed by drawing samples from a multivariate normal distribution centered at :math:`\boldsymbol{0}` with covariance :math:`\boldsymbol{\Sigma}`.
These tangent space samples are then mapped to the manifold by the Riemannian exponential at :math:`\boldsymbol{\mu}`.
Those manifold-valued samples are depicted as blue stars on the above image.

.. todo:: check if that has been published before?


.. _examples_gaussian_basic_example:
Example Code: ``examples/gaussian/gaussian.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/gaussian/gaussian.py
   :lines: 23-


.. _examples_gaussian_marginalization_conditioning:

Marginalization and Conditioning
--------------------------------

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/gaussian/marginalized_gaussian.*
          :alt: Marginalized Gaussian.

          Marginalized Gaussian.
     - .. figure:: images/gaussian/conditioned_gaussian.*
          :alt: Conditioned Gaussian at :math:`t=0.36`.

          Conditioned Gaussian at :math:`t=0.36`.

Marginalization using :meth:`pglfd.model.gaussian.Gaussian.margin` only keeps the elements of specific submanifolds in a Gaussian, dropping the other elements.
Conditioning with :meth:`pglfd.model.gaussian.Gaussian.condition`, in constrast, *conditions* the output on a specified input.

The result can be seen in the image above, where marginalization drops the time encoded in the Gaussian and conditioning lets us extract points based on time input.


Example Code: ``examples/gaussian/conditioned_gaussian.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/gaussian/gaussian_marg_cond.py
   :lines: 23-


.. _examples_gaussian_product:

Gaussian Product
----------------

.. figure:: images/gaussian/gaussian_product.*
   :alt: Riemannian Gaussian product on :math:`\mathcal{S}^2`.
   :width: 50%
   :align: center

   Riemannian Gaussian product on :math:`\mathcal{S}^2`.

The product of two Gaussian probability density functions itself is also a Gaussian distribution, it can also be calculated iteratively for the Riemannian Gaussian :cite:p:`zeestraten2017manifold`.
The above image shows the product of two Gaussians fitted on data segments (small red ellipse).
Note that in such *product of experts*, all constraints have to be satisfied approximately, leading to the smaller overall covariance.

In Python, the explicit function :meth:`pglfd.model.gaussian.gaussian_product` needs to be called while in C++, the :cpp:func:`pglfd::Gaussian::operator*` is overloaded, returning a :cpp:class:`pglfd::GaussianProduct`.
Just like in Eigen, this product by explicitly calling :cpp:func:`pglfd::GaussianProduct::eval` or when assigning to a :cpp:class:`pglfd::Gaussian`.


Example Code: ``examples/gaussian/gaussian_product.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/gaussian/gaussian_product.py
   :lines: 23-