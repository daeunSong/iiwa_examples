# iiwa_examples
iiwa examples provided by Ewha Glab

Clone the *moveit_visual_tools* repository of ours for better visualization result.
```shell
git clone -b glab/drawing https://github.com/daeunSong/moveit_visual_tools.git
```

Current code execution:
```shell
roslaunch iiwa_moveit moveit_planning_execution config_name:=moveit_reactive.rviz
```

(python)

```shell
rosrun iiwa_examples spawn_model.py
```
```shell
rosrun iiwa_examples plan_the_motion.py __ns:=iiwa
```


(c++)

```shell
rosrun iiwa_examples plan_motion __ns:=iiwa
```
