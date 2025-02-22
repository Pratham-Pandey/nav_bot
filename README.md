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
