# Realtime Optic Flow with ROS example

Installation: install as a standard catkin package:
    1. Put this folder in `catkin_ws/src`
    2. From catkin_ws, run `catkin_make`

# Run the demo

To play the demo videos under `demo_video`, which are compressed rosbag video files:
1. Edit the launch files so that the path (line 3, args=".....") match the path and name of the rosbag video files on your system. 
2. Run the following command from inside the demo_video folder: `roslaunch run_optic_flow_outside.launch`. If you get an error, check that the path is correct, and that you used the full path. It should pop up a window of the video feed.
3. Check which topic the video is on by running `rostopic list`. It should be `/camera/image_mono`
4. Run the optic flow node: `rosrun optic_flow_example optic_flow_lucas_kanade.py --topic='/camera/image_mono'`. This will open a window that shows the raw video feed, overlaid with red lines showing the optic flow at these pixels. The node also publishes the flow field data on the topic `/optic_flow`.
