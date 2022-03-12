

CQ-CAM Documentation
===================================

CQ-CAM is a CAM extension for generating and visualizing CNC milling toolpaths for CadQuery. 

Test out rendering
-----------------

.. cadquery::

    from cq_cam.job import Job
    from cq_cam.operations.profile import Profile
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    result = cadquery.Workplane("front").box(20.0, 20.0, 5)

    job_plane = result.faces('>X').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Profile(job=job, clearance_height=5, top_height=0, face=result.faces('<X'), offset=4, stepdown=-2.77)
    result = toolpath = visualize_task(job, op)
    #result.objects.append(toolpath)

Quick Links
------------------

  * :ref:`quickstart`
  * `CadQuery CheatSheet <_static/cadquery_cheatsheet.html>`_
  * :ref:`apireference`

Table Of Contents
-------------------

.. toctree::
    :maxdepth: 2

    intro.rst
    installation.rst
    quickstart.rst
    designprinciples.rst
    primer.rst
    sketch.rst
    assy.rst
    fileformat.rst
    examples.rst
    apireference.rst
    selectors.rst
    classreference.rst
    importexport.rst
    cqgi.rst
    extending.rst
    roadmap.rst



Indices and tables
-------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
