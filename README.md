⚠ ATTENTION ⚠
=============

A parallel project exists with a significantly larger feature set at the cost of having a hard
dependency to FreeCAD libraries: [ocp-freecad-cam](https://github.com/voneiden/ocp-freecad-cam)

Regardless of the dependency, the advantages with the approach of ocp-freecad-cam appear to be 
such that further development of cq-cam may not be relevant anymore. 

CQ-CAM
====

CQ-CAM aims to become a free, parametric CAM solution for
3-axis CNC mills closely integrating with CadQuery. 

[Documentation: https://cq-cam.readthedocs.io/en/latest/](https://cq-cam.readthedocs.io/en/latest/)

⚠ NOTE ⚠
===
Highly experimental at this stage. Expect bugs and always inspect the generated gcode.

2023-04-22: A major refactoring has been merged, likely breaking features that were present in v0.1. 
Proceed with extra caution if using the untsable branch!

Requirements and installation
---
* [cadquery=master=py3.10](https://github.com/CadQuery/cadquery)
* Optional visualization: [cq-editor=master=py3.10](https://github.com/CadQuery/CQ-editor)
* Optional 3D toolpaths: [opencamlib=2019.07](https://anaconda.org/conda-forge/opencamlib) (note, not available in PyPI)
* Python packages in requirements.txt / setup.py

### Install locally editable development version

Install cadquery and the optional deps (CQ-Editor, opencamlib).

Clone this repo, navigate to the source folder and run `pip install -e .`


### Install release version

As above, install cadquery and the optional deps (CQ-Editor, opencamlib) and run `pip install cq-cam`



Contributions
---
Feedback is welcome. Please discuss before proposing changes or opening PR's.


Roadmap to v1.0
---
The scope of v1.0 is open for the time being. Some possibilities are

* Climb/Conventional
* Bore Operation (helical drilling)
* Optimize rapid and cut order (opt-2? meanwhile if you need more optimized rapids check out https://gcode-sort.web.app/)
* Other strategies
* Ramps

Current features (v0.1)
----------------
See https://cq-cam.readthedocs.io/en/latest/operations.html for a showcase of different operations and features.

* 2.5D Profiling
* 2.5D Pocketing with ZigZag and Contour strategies
* 3D Surface with ZigZag strategy, using opencamlib
* Drill Operation
* Visualizing toolpaths in CQ-Editor (requires master version)
* G-Code generation
* Profiling tabs
* Optimize rapid and cut order (nearest neighbour)


Permanently outside of scope
---------------------------

* Simulation
* Lathes and 4D


