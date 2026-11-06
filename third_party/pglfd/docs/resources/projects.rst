.. _projects:

Projects
========

.. raw:: latex

   \begingroup
   % Intercept the item label [1] and pass nothing, removing the bullet
   \let\olditem\item
   \renewcommand{\item}[1][]{\olditem[]}



Algorithm Developments
----------------------

TODO projects developing novel algorithms implemented this lib



A Probabilistic Approach to Multi-Modal Adaptive Virtual Fixtures
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

moment matching in :ref:`gmm`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2024probabilistic`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2024probabilistic"
      :list: enumerated



Geometric and Model Priors in Motion Primitives
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Nadaraya-Watson implementation :ref:`nw`, kmp :ref:`kmp`, lqt :ref:`lqt`, see nonparametric examples: :ref:`examples_nonparametric`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2026geometric`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2026geometric"
      :list: enumerated



A Unified Framework for Probabilistic Dynamic-, Trajectory- and Vision-based Virtual Fixtures
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

velocity controllers :ref:`pgctrl`, parts of kernels :ref:`kernel`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2026unified`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2026unified"
      :list: enumerated



Passive Variable Impedance For Shared Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

passivity for controllers :ref:`pgctrl`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2026stabilizing`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2026stabilizing"
      :list: enumerated



Fast Non-Parametric Heteroscedastic Imitation Learning With Geometric Priors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

KMP implementation, ... :ref:`kmp` and :ref:`kernel` (but also :ref:`gp`), see nonparametric examples: :ref:`examples_nonparametric`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2026nonparametric`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2026nonparametric"
      :list: enumerated



Learning Geometry-Aware Virtual Fixtures From Sparse Demonstrations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

lqt :ref:`lqt`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2026lqt`

   .. bibliography:: ../refs_algorithms.bib
      :filter: key == "muehlbauer2026lqt"
      :list: enumerated




Applications
------------

TODO projects using this lib with no theoretical developments; can serve as reference on how to use it; space, industrial robotics etc.

TODO maybe compress those into different sections?



AI-enabled Cyber–Physical In-Orbit Factory - AI approaches based on digital twin technology for robotic small satellite production
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses kmp :ref:`kmp`, control :ref:`pgctrl` on a space robotic arm

.. container:: no-bullet

   **Paper** :cite:p:`leutert2024ai`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "leutert2024ai"
      :list: enumerated



RoboGrav - Towards Force Sensitive Space Manipulators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses kmp :ref:`kmp`, control :ref:`pgctrl` on a space robotic arm

.. container:: no-bullet

   **Paper** :cite:p:`maurenbrecher2024robograv`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "maurenbrecher2024robograv"
      :list: enumerated



Software for the SpaceDREAM Robotic Arm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses kmp :ref:`kmp`, control :ref:`pgctrl` on a space robotic arm

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2025software`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "muehlbauer2025software"
      :list: enumerated



Control System for a Torque-Sensitive Space Robotic Arm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses kmp :ref:`kmp`, control :ref:`pgctrl` on a space robotic arm

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2025control`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "muehlbauer2025control"
      :list: enumerated



Interactive robot skill adaptation framework based on imitation learning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses kmp :ref:`kmp`, control :ref:`pgctrl`

.. container:: no-bullet

   **Paper** :cite:p:`knauer2026interactive`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "knauer2026interactive"
      :list: enumerated



Toward Diverless UXO Recovery: Exploring Haptic Teleoperation in a Scaled Underwater Testbed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

uses lqt :ref:`lqt`, control :ref:`pgctrl`

.. container:: no-bullet

   **Paper** :cite:p:`werner2026toward`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "werner2026toward"
      :list: enumerated



A Shared Control Architecture for In-Space Satellite Servicing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


uses lqt :ref:`lqt`, control :ref:`pgctrl`

.. container:: no-bullet

   **Paper** :cite:p:`muehlbauer2027shared`

   .. bibliography:: ../refs_projects.bib
      :filter: key == "muehlbauer2027shared"
      :list: enumerated


.. raw:: latex

   \endgroup