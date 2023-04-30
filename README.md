# Introduction
For the Robothon 2023 Challenge, the vision prepossessing required three different tasks:
- localization of the board;
- reading the M5 screen for the slider setting.
- reading the multimeter screen for BYOD part of the challenge has been implemented in this package.

# Camera launch
The camera used for this task is realsense D435i, attached to the end effector of the Franka Emika arm.
```bash
roslaunch realsense2_camera rs_camera.launch camera:=d435_camera
```

# Slider triangles detection
By detecting the screen and the indicators of the current slider position and the desired position and scaling the distances to the relative position command for the slider on the slot, we could accomplish this task.

```bash
rosrun hrii_board_localization screen_read.py
```

## dependencies:
- OpenCV_Version==4.7.0
- rospy
- hrii_robothon_msg

# Voltage measurement
```bash
rosrun hrii_board_localization text_read.py
```
## dependencies:
- OpenCV_Version==4.7.0
- rospy
- hrii_robothon_msg
- google_cloud_vision
- matplotlib
- tkinter
- PIL

### Notes
- A google vision API is required to run the task, make sure the credential for the mentioned API is set up correctly.


<div align="center">
    <img src="resources/clideo_editor_425c5d366b5149198acba5f50786bdff.mp4" width="600" controls>
</div>

<!--
![Image Description](https://gitlab.iit.it/hrii/projects/robothon/hrii_board_localization/-/blob/master/resources/text_read.png) -->
