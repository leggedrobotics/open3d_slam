.. _open3d_localization_ref:

============
Localization
============

Open3D slam can be initialized with a map and run in localization mode. There are two different localization modes:

1. Pure localization - initial map stays intact and no new scans are merged into the map. It is active with the parameter ``is_use_map_initialization`` is set to true and ``is_merge_scans_into_map`` is set to false.

2. Localization with map building - new scans are merged into the initial map. It is active if ``is_use_map_initialization`` and ``is_merge_scans_into_map`` are both set to true.

Localization uses ICP which is a local method and one needs to provide a good initial guess for the pose. One way of doing so is directly in the parameter file, however it can also be done interactively at runtime (see links below).
Parameters used for localization are given below:

map_intializer:
    ``is_initialize_interactively`` - whether to enable initialization at runtime.
    
    ``frame_id`` - frame in which you are initializing open3d_slam, relevant for interactive initialization
    
    ``pcd_file_path`` - full path to your map that you want to localize against
    
    ``init_pose`` - here you specify initial pose, translation xyz is in meters and roll, pitch, yaw are in degrees.
      

Sometimes it is useful to initialize the map interactively, i.e. start the robot, look at the measurements and try a few poses until the raw measurment is finally aligned with the map.

.. warning::
    While you are initializing the pose interactively, both odom to sensor frame and map to odom frame transformations can jump!



You can initialize the pose interactively with the 2D pose estimate tool in Rviz, which is shown in the video below:
`initialize using 2D pose estimate <https://youtu.be/tvDPKHIizdI>`_ .

The white pointcloud is the raw measurement; once it is nicely aligned with the map, your job is done, and you are ready to start the mapping. 
You can do this by calling the Trigger service "initialize_slam". After that, the mapping will start and the scans will be
inserted into the map if ``is_merge_scans_into_map`` is set to true.



Alternatively, you can use the 3d interactive marker which is shown the other video below:
`initialize with interactive marker <https://youtu.be/ePI3SvR3zpw>`_ . 

You have to enable the interactive marker first. Then you can drag the marker to
the estimated robot pose, and then right click on the marker and select "Set pose". If the icp diverges, try better aligning the marker and doing it again. As you drag the marker, the blue pointcloud shows what the raw measurement
would look like if the robot was at that given pose. Essentially, the blue pointcloud should fit nicely against the map. Once you have it aligned nicely,
right click and select "initialize slam" which will start the mapping. Scans will be
inserted into the map if ``is_merge_scans_into_map`` is set to true.






TODO localization example with a dataset
