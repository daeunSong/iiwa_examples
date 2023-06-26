# iiwa_examples

*Tested on **Ubuntu 18.04** with **ROS Melodic**.*

iiwa examples provided by Ewha Glab. These examples depend on [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack).

## Demo
Once the connection is established, run the scripts. 
```sh
rosrun iiwa_examples move_spline_demo
```

If you wish to make use of MoveIt! to operate iiwa, you also need to run the followings.
```sh
roslaunch iiwa_moveit demo.launch 
rosrun iiwa_examples updown_moveit
```
