.. _examples_tp:
  
Task-Parameterized Models
=========================

Task-Parameterized Models enable the generalization of skills to novel task frames.
`pglfd` implements task parameterization both on :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` and on other :ref:`examples_manifolds_product` :cite:p:`muehlbauer2026nonparametric`.


Cartesian Task-Parameterization
-------------------------------

.. list-table::
   :widths: 50 50
   :header-rows: 0
   :class: image-table nocolorrows

   * - .. figure:: images/tp/cartesian.*
          :alt: Cartesian task parameterization without target frame rotation.

          No target frame rotation.
     - .. figure:: images/tp/cartesian_90deg.*
          :alt: Cartesian task parameterization with target frame rotation of 90°.

          Target frame rotated by :math:`90°`.

In Cartesian task parameterization, all models are in :math:`\mathbb{R}^3 \times \mathrm{SO}(3)`.
The above images (example taken from :cite:p:`muehlbauer2026nonparametric`) show two instantiations of a task parameterized model learned for two task frames depicted by longer coordinate systems.
The ``start_model`` encodes motion in the right coordinate frame, while the ``end_model`` encodes the motion in the left coordinate frame.
It can be seen that the increasing / decreasing covariance leads to a transition between both models in ``fused``.

While in the above case we use a task-parameterized :class:`pglfd.model.kmp.KMP`, the :class:`pglfd.model.tp.TaskParameterizedModel` supports any model implementing :class:`pglfd.model.gaussian_regressor.GaussianRegressor`.
The :ref:`Example Code <examples_tp_tp_example_code>` shows how such models can be learned by first transforming the data into the respective coordinate frames, fitting individual models and composing them in a task-parameterized model which can then be queried.


.. _examples_tp_tp_example_code:
Example Code: ``examples/task_parameterization/cartesian.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/task_parameterization/cartesian.py
   :lines: 23-



Multi-Manifold Task-Parameterization
------------------------------------

.. todo:: supply image and below code

The :class:`pglfd.model.tp.TaskParameterizedModelManif` supports a fusion in any of the :ref:`examples_manifolds_product` :cite:p:`muehlbauer2026nonparametric`.
The first model supplied is supposed to be expressed in the respective manifold, while all other models either have to be in :math:`\mathbb{R}^3 \times \mathrm{SO}(3)` or are transformed in this manifold.
This enables leveraging the geometry of the first manifold, e.g., to have low stiffness in one DoF.


Example Code: ``examples/task_parameterization/multi_manifold.py``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../python/examples/task_parameterization/multi_manifold.py
   :lines: 23-