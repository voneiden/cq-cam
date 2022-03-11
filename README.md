CQ-CAM
====

CQ-CAM aims to become a free, parametric CAM solution for
3-axis CNC mills closely integrating with CadQuery. 

⚠ NOTE ⚠
===
Highly experimental at this stage. This is NOT ready for general use or even the real world.

Requirements
---

* Visualization: cq-editor=master=py3.10
* 3D toolpaths: opencamlib=2019.07
* Python packages in requirements.txt


Contributions
---
Feedback is welcome, but please no spontaneous code contributions.
Open an issue, discuss and get an approval before submitting a PR.


Roadmap to first release
---
The initial pace has been pretty good. The first release might be doable 
during March 2022. 

#### To-do
* Drill Operation
* Release package preparation
* Basic documentation


#### In Progress
* Migrating code from PoC to production quality
  * Tidying and unifying code
  * Adding integration tests
  
#### Done
* 2.5D Profiling
* 2.5D Pocketing with ZigZag strategy
* Contour strategy
* 3D Surface with ZigZag strategy, using opencamlib
* Visualizing toolpaths in CQ-Editor (requires master version)
* G-Code generation
* Profiling tabs
* Adding unit tests and refactor to more functional code to simplify testing
* Optimize rapid and cut order (nearest neighbour)

### After the first release?
* Climb/Conventional
* Bore Operation (helical drilling)
* Optimize rapid and cut order (opt-2? meanwhile if you need more optimized rapids check out https://gcode-sort.web.app/)
* Other strategies
* Ramps


Outside of scope
---

* Simulation
* Lathes and 4D


