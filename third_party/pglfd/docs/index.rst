``pglfd`` - Probabilistic and Geometric Learning from Demonstration
===================================================================

.. warning::

   You are looking at a prerelease version of `pglfd`.
   While the code is tested thoroughly, the documentation is unfinished and may not be useful or even incorrect in some places.


``pglfd`` provides algorithms for geometric and probabilistic learning from demonstration and control of robots.
Most methods are implemented for both `Python` and `C++`, where the former also uses `C++` in the backend to speed up computations for commonly used manifolds.
`Matlab` and `Simulink` implementations are provided for geometric methods required in the geometric impedance and velocity control of robots.

Only minimal dependencies (``numpy`` for `Python`, ``Eigen`` for `C++`) are required.
See :ref:`installation` for details and :ref:`licenses` for reusing the code.

The documentation first presents relevant algorithms and control methods that can be implemented using ``pglfd``, followed by a detailed reference of all implemented methods.
We suggest to read the paper describing this library :cite:p:`muehlbauer2026pglfd` for a more detailed understanding:

.. code-block:: text

   @Article{muehlbauer2026pglfd,
   author   = {Mühlbauer, Maximilian and Authors, More and Silvério, João},
   title    = {pglfd: A Library for Probabilistic and Geometric Learning from Demonstration and Control},
   journal  = {IEEE Robotics and Automation Practice (under review)},
   year     = {2026},
   }

Please cite this work when using ``pglfd``, as well as relevant papers describing the methods.
An overview of works leading to the creation of this library as well as of works using this library can be found under :ref:`projects`.

.. raw:: latex

   \part{Algorithms}

.. toctree::
   :maxdepth: 2
   :caption: Algorithms

   algorithms/manifolds
   algorithms/gaussian
   algorithms/parametric
   algorithms/nonparametric
   algorithms/tp
   algorithms/control


.. raw:: latex

   \part{Geometry}

.. toctree::
   :maxdepth: 2
   :caption: Geometry

   modules/geom/so2
   modules/geom/s2
   modules/geom/so3
   modules/geom/se3


.. raw:: latex

   \part{Manifolds}

.. toctree::
   :maxdepth: 2
   :caption: Manifolds

   modules/manif/manifold
   modules/manif/conversions
   modules/manif/jacobian
   modules/manif/util


.. raw:: latex

   \part{Models}

.. toctree::
   :maxdepth: 2
   :caption: Models

   modules/model/kmeans
   modules/model/gaussian
   modules/model/gmm
   modules/model/gmr
   modules/model/kernel
   modules/model/gp
   modules/model/kmp
   modules/model/lqt
   modules/model/nw
   modules/model/tp


.. raw:: latex

   \part{Control}

.. toctree::
   :maxdepth: 2
   :caption: Control

   modules/ctrl/pgtrafo
   modules/ctrl/pgmanif
   modules/ctrl/pgctrl


.. raw:: latex

   \part{Resources}

.. toctree::
   :maxdepth: 2
   :caption: Resources

   modules/visualization/matplotlib
   resources/installation
   resources/projects
   resources/licenses
   resources/patents

.. only:: html

   .. toctree::

      resources/bibliography

.. only:: html

   Indices and tables
   ==================

   * :ref:`genindex`
   * :ref:`modindex`
   * :ref:`mat-modindex`

.. only:: latex

   .. bibliography:: refs_algorithms.bib

   .. bibliography:: refs_projects.bib
      :labelprefix: P
