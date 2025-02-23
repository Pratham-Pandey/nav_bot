# Optimize the Navigation Pipeline
1. local_costmap:
  * update_frequency and publish_frequency:
  *  Before updating these, when dynamic objects leaves a position on the map, there presence in map takes a while to get updated.
  *  Increasing the frequency resulted in faster map updation. But the issue is not copletely resolved because the CPU is at 99% usage.

2. FollowPath(Local Planner):
   * samples: Increased vx_samples to 25 from 20. Reduces vy_samples to 2 from 5. Reduces vheta_samples to 20 from 15.
   * reduced sim_time to 1.2 from 1.7 to reduce computation load. 
   * BaseObstacle.scale: Increased it to 0.04 from 0.02 to prioritize obstacle avoidance.
    
2.  gobal_costmap/inflation_radius:
  *  Changing  it from 0.55 to 0.15 enabled robot to move through tight spaces although with increased risk of collision.
  *  The robot selected another shortest path to the same goal due to this. 

3  planner_server/
  * expected_planner_frequency: increased frequency from 20Hz to 30Hz for better resopnse to dynamic obstacles.
  * tolerance: Increased to 1.0 from 0.5. Now the goal is considered reached even if the robot is a bit away from it.
  * use_astar: Did not yeild any benefit. It made the path sink towards the obstacles.

4. smoother_server:
  * tolerance: Increased it to 1.0e-5  from 1.0e-10 to improve performance. Didnt had much effect on path but computational load was reduced.
  * max_its: reduced it to 200 from 1000. Didnt had much effect on path but computational load was reduced.

5. velocity_smoother:
   * max_velocity: Increased max_velocity in x directn to 1.0 from 0.26 for faster movement.
   * max_accel: Reduced max acceleration in x direction from 2.5 to 1.9 and angular acceleration(z) to 2.5 from 3.2 for smoother movement.


#  Output Video

**Before Optimization:**  https://drive.google.com/file/d/1nUfJM8i7tJhad4OAIFyYR3xVR2ikuEVn/view?usp=drive_link  
**After Optimization:**  https://drive.google.com/file/d/1uuLApxWm7mVJ8jArFPF9ZyOWEgPz4xMt/view?usp=sharing

**NOTE:** Due to low specs of the laptop, the whole nav2 task is running slowly. That is the reason why I have made the video using an external camera instead of a screen recorder. The video is shaking initially, later it gets a bit stablized. 


# Steps To Run Locally

>  [!NOTE]
> Change the file path of parameter **"map_file_name:"** in the **mapper_params_online_async.yaml** in the following way:
> ```{path_where_cloned}/nav_bot/maze2_serialize1```


Clone the project
```bash
  git clone https://github.com/Pratham-Pandey/nav_bot.git
```
Go to the project directory
```bash
  cd nav_bot
```

Build Project and source 
```bash
  colcon build --symlink-install
  source install/setup.bash
```



Run nav_bot
```bash
  ros2 launch nav_bot launch.py use_sim_time:=true
```

Run Publisher Node(Used to input goal location)
```bash
  ros2 run nav_bot my_pub
```
