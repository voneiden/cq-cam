

CQ-CAM Documentation
===================================

CQ-CAM is a CAM extension for generating and visualizing CNC milling toolpaths for CadQuery. 

Test out rendering
------------------

.. cadquery::

    from cq_cam.job import Job
    from cq_cam.operations.profile import Profile
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    wp = cadquery.Workplane("front").box(20.0, 20.0, 5)

    job_plane = wp.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Profile(job=job, clearance_height=5, top_height=0, wp=wp.wires('<Z'))
    toolpath = visualize_task(job, op, as_edges=True)
    #result = cadquery.Assembly().add(wp).add(toolpath)
    #result = wp.edges()
    result = wp
    result.objects += toolpath



Table Of Contents
-------------------

.. toctree::
    :maxdepth: 2

    installation.rst
    quickstart.rst
    operations.rst
    algorithms.rst
    classreference.rst



Indices and tables
-------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
