# ROS 2 Bag – Quick Cheat Sheet

A minimal reference for recording, inspecting, and replaying ROS 2 bags.

---

## Basics

### List available topics

```bash
ros2 topic list
```

### Echo a topic (quick sanity check)

```bash
ros2 topic echo /topic_name
```

### Recording

- **Record all topics**  
  ```bash
  ros2 bag record -a
  ```  

- **Record specific topics**  
  ```bash
  ros2 bag record /tf /tf_static /odom /cmd_vel
  ```  

  ```bash
  ros2 bag record -e '^/tf(_static)?$|^/zed/zed_node/(?!imu/|odom$|pose$|pose/status$).+'


  ```  

- **Set output folder name**  
  ```bash
  ros2 bag record -a -o my_bag
  ```  

- **Record for a fixed duration (seconds)**  
  ```bash
  ros2 bag record -a --duration 30
  ```  

- **Split bag files by size (MiB)**  
  ```bash
  ros2 bag record -a --max-bag-size 2048
  ```  

- **Common robot topic set**  
  ```bash
  ros2 bag record \
    /tf \
    /tf_static \
    /odom \
    /imu \
    /cmd_vel \
    /joint_states \
    /scan
  ```  

### Inspecting Bags

- **Show bag info (topics, types, duration, message count)**  
  ```bash
  ros2 bag info my_bag
  ```  

### Playback

- **Play a bag**  
  ```bash
  ros2 bag play my_bag
  ```  

- **Play slower / faster**  
  ```bash
  ros2 bag play my_bag --rate 0.5
  ros2 bag play my_bag --rate 2.0
  ```  

- **Loop playback**  
  ```bash
  ros2 bag play my_bag --loop
  ```  

- **Play only selected topics**  
  ```bash
  ros2 bag play my_bag --topics /tf /odom
  ```  

- **Start playback paused**  
  ```bash
  ros2 bag play my_bag --pause
  ```  

- **Unpause:**  
  ```bash
  ros2 service call /rosbag2_player/toggle_paused std_srvs/srv/Trigger "{}"
  ```ros2 bag play my_bag --rate 0.5
ros2 bag play my_bag --rate 2.0
Loop playback
ros2 bag play my_bag --loop
Play only selected topics
ros2 bag play my_bag --topics /tf /odom
Start playback paused
ros2 bag play my_bag --pause
Unpause:

ros2 service call /rosbag2_player/toggle_paused std_srvs/srv/Trigger "{}"