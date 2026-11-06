.. _tp:
  
Task-Parameterized Model
========================

TODO allow to combine outputs of different methods implementing GaussianRegressor

TODO document how GaussianRegressor, GaussianRegressorR3SO3 interact


Python API
----------

.. automodule:: pglfd.model.tp

.. autoclass:: TaskParameterizedModel
   :members:
   :inherited-members:

   .. automethod:: __call__

.. autoclass:: TaskParameterizedModelManif
   :members:

   .. automethod:: __call__

.. automodule:: pglfd.model.gaussian_regressor

.. autoclass:: GaussianRegressor
   :members:

   .. automethod:: __call__


C++ API
-------
.. doxygenclass:: pglfd::TaskParameterizedModel
   :members:

.. doxygenclass:: pglfd::TaskParameterizedModelManif
   :members:

.. doxygenclass:: pglfd::GaussianRegressor
   :members:

.. doxygenclass:: pglfd::GaussianRegressorR3SO3
   :members:
