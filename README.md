CQ-CAM
====

CQ-CAM aims to become a free, parametric CAM solution for
3-axis CNC mills closely integrating with CadQuery. 

Roadmap
---

* [DONE] PoC - make sure it's possible to visualize 
  toolpaths in CQ-Editor and experiment with profile toolpaths
* [In progress] Foundations - implement the base features 
  before rushing in to do other operations. This includes things
  like utility functions for converting OpenCASCADE shapes into 
  easily digestible geometry for toolpath generation, cw/ccw logic
  (spindle rotation, climb/conventional milling), dealing with arc motions
  optimizing rapid movements and so on. The goal is to end up with a reliable Profile operation
* Unit test foundations
* [In progress] Library packaging (probably conda based 
  because of future dependency to opencamlib)
* Pocketing - implement pocketing operation with various strategies
  * Zigzag
  * Contour
  * ???
    
* Holes - drilling holes, countersinks, etc.
* Extending features such as tabs and ramps
* 3D CAM via integration with opencamlib. 
* G-Code translations to weird dialects?



Outside of scope
---

* Simulation
* Lathes and 4D


Contributions
---
Feedback is welcome. Please open an issue and discuss 
before submitting a PR.