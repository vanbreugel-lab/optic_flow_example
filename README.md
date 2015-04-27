Installation: install as a standard catkin package:
    1. Put this folder in catkin_ws/src
    2. From catkin_ws, run catkin_make

To play the demo videos, which are compressed rosbag video files, first you will have to edit the launch files so that the path (line 3, args=".....") match the path and name of the rosbag video files. Then run the following command from inside the demo_video folder:

roslaunch run_optic_flow_outside.launch

If you get an error, check that the path is correct!

To view the video, run the following command in a new terminal window:

rosrun image_view image_view image:=/camera/image_mono

To run the optic flow node, run the following command in a new terminal window:

rosrun optic_flow_example optic_flow_lucas_kanade.py

This will open a window that shows the raw video feed, overlaid with red lines showing the optic flow at these pixels. The node also publishes the flow field data on the topic /optic_flow.
