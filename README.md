# Optimize the Navigation Pipeline
1. Changing locaal costmap **update_frequency** and **publish_frequency**:
  *  Before updating these, when dynamic objects leaves a position on the map, there presence in map takes a while to get updated.
  *  Increasing the frequency resulted in faster map updation. But the issue is not copletely resolved because the CPU is at 99% usage.

    
2.  gobal_costmap/inflation_radius:
  *  Changing  it from 0.55 to 0.20 enabled robot to move through tight spaces although with increased risk of collision.
  *  The robot selected another shortest path to the same goal due to this. 
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
