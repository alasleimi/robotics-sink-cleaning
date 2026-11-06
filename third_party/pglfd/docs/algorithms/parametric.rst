.. _examples_parametric:
  
Parametric Models
=================

Parametric models fit model parameters based on training data.
This chapter briefly explains the models implemented in `pglfd`.

.. todo:: handwriting dataset citation!


K-Means
-------

.. figure:: images/parametric/kmeans.*
   :width: 50%
   :align: center
   :alt: K-Means fitting.

   K-Means clustering on :math:`\mathcal{S}^2` with :math:`10` clusters.

Riemannian K-Means uses the distance measure :eq:`examples_manifolds_weighted_distance` with :math:`\boldsymbol{A} = \boldsymbol{I}` to calculate distances to cluster centers.
After finding the closest centers, the :ref:`fréchet mean <util>` is used to calculate the updated center means.
In the figure above, we use the :class:`pglfd.model.kmeans.KMeans` class to fit a Riemannian K-Means model with :math:`10` cluster centers on the handwritten letter ``B`` data mapped to :math:`\mathcal{S}^2`.


Example Code: ``examples/parametric/kmeans.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/parametric/kmeans.py
   :lines: 23-


Gaussian Mixture Model
----------------------

A Riemannian Gaussian Mixture Model fits multiple :ref:`Riemannian Gaussians <examples_gaussian>` as a mixture of experts model.
The probability density function of a GMM with :math:`N` components evaluates to

.. math::
   :label: examples_parametric_gmm_pdf

   p(\boldsymbol{x}) = \sum_i^N \pi_i~\mathcal{N}\left(\boldsymbol{x}\vert\boldsymbol{\mu}_i,\boldsymbol{\Sigma}_i\right)

where :math:`\pi_i` are the prior probabilities and :math:`\boldsymbol{\mu}_i` mean and :math:`\boldsymbol{\Sigma}_i` covariance of the Gaussian components.


Fitting a GMM and sampling
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/parametric/gmm_fit.*
          :alt: GMM fitting.

          GMM on :math:`\mathcal{S}^2` with :math:`10` Gaussians.
     - .. figure:: images/parametric/gmm_sample.*
          :alt: GMM samples.

          Drawing :math:`50` samples from the GMM on :math:`\mathcal{S}^2`.

Similarly to K-Means, a GMM is fitted using an expectation maximization algorithm.
Initializing using, e.g., the K-Means clusters, the expectation step assigns each data point to its closest Gaussian while the maximization step recalculates the Gaussians, thus maximizing :eq:`examples_parametric_gmm_pdf` for the data.
This can be performed using :meth:`pglfd.model.gmm.GMM.fit` method.

Similarly to :ref:`sampling from a single Gaussian <examples_gaussian_sampling>`, we can also sample from a GMM with the :meth:`pglfd.model.gmm.GMM.sample` method.
This method first determines the Gaussian to sample from based on its prior and then samples from this Gaussian.
Both fitted GMM and sampled points can be seen in the image above.


Expectation of Samples
^^^^^^^^^^^^^^^^^^^^^^

The method :meth:`pglfd.model.gmm.GMM.expectation` calculates raw values of :eq:`examples_parametric_gmm_pdf`, i.e. without performing the sum.
Summing over the rows of the result returns the likelihood of each data point.


Marginalization
^^^^^^^^^^^^^^^

Marginalization of a GMM using :meth:`pglfd.model.gmm.GMM.margin` :ref:`marginalizes every Gaussian <examples_gaussian_marginalization_conditioning>` inside the GMM.


Approximating a Single Gaussian
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/parametric/product_gaussian.*
          :alt: Product of Gaussians.

          Product of two Gaussians on :math:`\mathcal{S}^2`.
     - .. figure:: images/parametric/mixture_gaussian.*
          :alt: Mixture of Gaussians.

          Mixture of two Gaussians on :math:`\mathcal{S}^2`.

By nature, GMMs are multi-modal and as such not well suited for problems where a single mean and covariance are required.
A unimodal Gaussian can be approximated by matching first (mean) and second (covariance) moments using :meth:`pglfd.model.gmm.GMM.approximate_single_gaussian`, with the Riemannian case described in :cite:p:`muehlbauer2024probabilistic`.
In contrast to a :ref:`examples_gaussian_product`, this is a *mixture of experts* where any of the conditions has to satisfied approximately.

The difference can be clearly seen in the above image: While the Gaussian product results in a different mean location and small covariance, the mean of the mixture lies centered between both Gaussians.
The much bigger covariance corresponds to the matching of second moments, where the enlargened covariance captures the fact that both original means are far apart.


Example Code: ``examples/parametric/gmm.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/parametric/gmm.py
   :lines: 23-


.. _examples_parametric_gmr:

Gaussian Mixture Regression
---------------------------

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/parametric/gmr_coarse.*
          :alt: Coarse GMR.

          Coarse GMR on :math:`\mathcal{S}^2` with :math:`5` Gaussians.
     - .. figure:: images/parametric/gmr_fine.*
          :alt: Fine GMR.

          Fine GMR on :math:`\mathcal{S}^2` with :math:`10` Gaussians.

Gaussian Mixture regression extends :ref:`Gaussian Conditioning <examples_gaussian_marginalization_conditioning>` from a single Gaussian to a GMM, calculating the conditional distribution

.. math::
   :label: examples_parametric_gmr_pdf

   p(\boldsymbol{y} \vert \boldsymbol{x}) = \sum_i^N \pi_i~\mathcal{N}\left(\boldsymbol{y}\vert\boldsymbol{\mu}_{i\vert\boldsymbol{x}},\boldsymbol{\Sigma}_{i\vert\boldsymbol{x}}\right).

The above figure shows two instantiations of Riemannian GMR, where a higher number of Gaussian components leads to a more precise reproduction at a computational cost.
Note that a too high number of Gaussian components leads to overfitting.


Example Code: ``examples/parametric/gmr.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/parametric/gmr.py
   :lines: 23-



Linear Quadratic Tracking
-------------------------

.. list-table::
   :widths: 33 33 33
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/parametric/lqt_coarse.*
          :alt: Coarse LQT.

          Coarse LQT (:math:`R = \mathbf{I} \cdot 1e-4`).
     - .. figure:: images/parametric/lqt_medium.*
          :alt: Medium LQT.

          Medium LQT (:math:`R = \mathbf{I} \cdot 2e-5`).
     - .. figure:: images/parametric/lqt_fine.*
          :alt: Fine LQT.

          Fine LQT (:math:`R = \mathbf{I} \cdot 5e-6`).

Riemannian Linear Quadratic Tracking :cite:p:`muehlbauer2026lqt` tracks probabilistic via points at specified times with minimal interventionary control, supplying a probabilistic state sequence.
An LQT can be fit using the :meth:`pglfd.model.lqt.LQT.fit` method.
The above images show the result for different tracking cost - the lower the control command cost, the more accurate is the tracking.
Probabilistic via points are extracted using :ref:`examples_parametric_gmr`.


Example Code: ``examples/parametric/lqt.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/parametric/lqt.py
   :lines: 23-