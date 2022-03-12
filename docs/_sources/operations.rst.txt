##########
Operations
##########

This page describes how to use the various operations provided by the library.

Profile
=========
:class:`cq_cam.operations.profile.Profile` generates 2.5D profile toolpaths.
It can be used for both outer and inner profiles.

Outer profile
-------------
.. cadquery::

    from cq_cam.job import Job
    from cq_cam.operations.profile import Profile
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    result = cadquery.Workplane("front").box(20.0, 20.0, 5)

    job_plane = result.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Profile(job=job, clearance_height=5, top_height=0, wp=result.wires('<Z'))
    toolpath = visualize_task(job, op, as_edges=True)
    result.objects += toolpath

Inner profile
-------------
.. cadquery::

    from cq_cam.job import Job
    from cq_cam.operations.profile import Profile
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    result = cadquery.Workplane("front").box(20.0, 20.0, 1).faces('>Z').workplane().rect(10, 10).cutThruAll()

    job_plane = result.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Profile(job=job, clearance_height=5, top_height=0, wp=result.faces('<Z'), face_offset_outer=None, face_offset_inner=-1)
    toolpath = visualize_task(job, op, as_edges=True)
    result.objects += toolpath

Tabs
----
Often when profiling it is not ideal to cut the whole profile all the way to avoid the cutout becoming loose and getting
damaged by the milling bit. Tabs are one way to deal with this. Two tabbing algorithms are provided.
WireTabs works on a wire and EdgeTabs works on each edge of a wire independently.
Because EdgeTabs works on individual edges it spaces the tabs evenly on the edge length and is therefore more likely to
generate a more aesthetically pleasing result. WireTabs is more suited for situations where even spacing is critical on the whole
wire length.


EdgeTabs
********
:class:`cq_cam.operations.tabs.EdgeTabs` generates tabs on each edge separately.
Edges can be filtered by type (line, circle, arc) and tabs can be placed by count,
spacing or manually defined positions.

WireTabs
********
:class:`cq_cam.operations.tabs.WireTabs` generates tabs on a wire.
Tabs can be placed by count, spacing or manually defined positions.




Pocket
=========
Pockets come in two variants. Closed pockets have no open edges so the tool stays always inside the outer boundary.
Open pockets may have open sides where the tool needs to travel outside of the outer boundary. These two cases
require a slightly different approach but both are fully supported. Additionally pocketing comes with a toolpath
strategy. Two strategies are currently implemented: zigzag and contour.

Closed pockets
--------------

.. cadquery::

    from cq_cam.job import Job
    from cq_cam.operations.pocket import Pocket
    from cq_cam.commands.base_command import Unit
    from cq_cam.visualize import visualize_task

    result = cq.Workplane("front").box(20.0, 20.0, 2).faces('>Z').workplane().rect(15, 15).cutBlind(-1)

    job_plane = result.faces('>Z').workplane()
    job = Job(job_plane, 300, 100, Unit.METRIC, 5)
    op = Pocket(job=job, clearance_height=5, top_height=0, wp=result.faces('<Z[1]'))
    toolpath = visualize_task(job, op, as_edges=True)
    result.objects += toolpath

Open pockets
------------



Drill
========


3D Surface
==========

