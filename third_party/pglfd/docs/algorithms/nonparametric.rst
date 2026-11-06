.. _examples_nonparametric:
  
Nonparametric Models
====================

Nonparametric models predict an output directly using training data, without first fitting a model.
This chapter briefly explains the models implemented in `pglfd`.

Kernels
-------

Kernels are used to measure the similarity between data points; they usually output a scalar similarity value.
`pglfd` provides the RBF kernel and the Matérn family of kernels in :mod:`pglfd.model.kernel`, both with scalar output and the diagonal formulation of :cite:p:`muehlbauer2026nonparametric`.
In :ref:`examples_nonparametric_gp` and :ref:`examples_nonparametric_kmp`, diagonal kernels enable the prediction of a matrix-valued epistemic uncertainty opposed to the normally only scalar-valued prediction.


Example Code: ``examples/nonparametric/kernels.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/nonparametric/kernels.py
   :lines: 23-


Nadaraya-Watson Regression
--------------------------

.. list-table::
   :widths: 33 33 33
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/nonparametric/nw_01.*
          :alt: NW regression.

          Kernel with :math:`l = 0.01`.
     - .. figure:: images/nonparametric/nw_05.*
          :alt: NW regression.

          Kernel with :math:`l = 0.05`.
     - .. figure:: images/nonparametric/nw_1.*
          :alt: NW regression.

          Kernel with :math:`l = 0.1`.

Nadaraya-Watson regression is a nonparametric kernel regression method, calculating a weighted influence of data points based on the input.
The Riemannian variant of :cite:p:`muehlbauer2026geometric` furthermore provides an aleatoric uncertainty estimate using a weighted variant of the algorithm used for calculating the covariance when fitting a Riemannian Gaussian from data points.
The algorithm is implemented in :class:`pglfd.model.nw.NadarayaWatson`; the above image showcases the prediction result for different kernel length scales.
With larger length scale, prediction gets smoother, but also more coarse.


Example Code: ``examples/parametric/nw.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/nonparametric/nw.py
   :lines: 23-


.. _examples_nonparametric_gp:

Gaussian Processes
------------------

.. list-table::
   :widths: 33 33 33
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/nonparametric/gp_01.*
          :alt: GP regression.

          Kernel with :math:`l = 0.01`.
     - .. figure:: images/nonparametric/gp_05.*
          :alt: GP regression.

          Kernel with :math:`l = 0.05`.
     - .. figure:: images/nonparametric/gp_1.*
          :alt: GP regression.

          Kernel with :math:`l = 0.1`.

Gaussian Processes implemented in :class:`pglfd.model.gp.GaussianProcess` model a probabilistic function for which uncertain measurements with constant covariance (homoscedasticity) are available.
The model predicts both mean and epistemic uncertainty.
The implementation in `pglfd` shares much code with :ref:`examples_nonparametric_kmp` which assume different covariance per data point (heteroscedascity).

The above figure shows the behavior of Gaussian Processes for different kernel length scales - for very small length scales, the process converges to a previously set *process mean* with growing covariance.
For larger length scales and closer proximity to the data, this uncertainty decreases and the process interpolates between data points.


Example Code: ``examples/parametric/gp.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/nonparametric/gp.py
   :lines: 23-


.. _examples_nonparametric_kmp:

Kernelized Movement Primitives
------------------------------

.. list-table::
   :widths: 33 33 33
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/nonparametric/kmp_01.*
          :alt: KMP regression.

          Kernel with :math:`l = 0.01`.
     - .. figure:: images/nonparametric/kmp_05.*
          :alt: KMP regression.

          Kernel with :math:`l = 0.05`.
     - .. figure:: images/nonparametric/kmp_1.*
          :alt: KMP regression.

          Kernel with :math:`l = 0.1`.

Riemannian Kernelized Movement Primitives :cite:p:`muehlbauer2026nonparametric` implemented in :class:`pglfd.model.kmp.KMP` extend Gaussian Processes, allowing for the specification of different covariances per data point.
This enables the output of an uncertainty containing both aleatoric and epistemic uncertainties - close to the data points, the model outputs the aleatoric uncertainty from supplied probabilistic data points, while the epistemic uncertainty estimate increases when moving away from known points.

The difference to :ref:`examples_nonparametric_gp` can be seen in the figure above - while the two plots for small lenth scales look very similar to the GP prediction, the covariance prediction for the largest length scale accurately reproduces the variance of trajectories whereas it almost vanishes for the Gaussian Process.


Example Code: ``examples/parametric/kmp.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/nonparametric/kmp.py
   :lines: 23-