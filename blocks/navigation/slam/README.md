# 2D LiDAR SLAM (Slam Toolbox)

This block provides real-time mapping and localization using a 2D LiDAR (e.g., LD19). It uses the **Slam Toolbox** to create a 2D Occupancy Grid map.

## 🚀 How to Run
1.  Connect your LiDAR (ensure it's on `/dev/ttyUSB0`).
2.  Launch the SLAM process:
    ```bash
    ros2 launch xpi_navigation slam.launch.py
    ```
3.  Open **Rviz2**.
4.  Add a **Map** display and set the topic to `/map`.
5.  Set the **Fixed Frame** to `map`.

## 💾 Saving the Map
Once you are happy with the map, save it using the `map_saver` tool from `nav2_map_server`:
```bash
ros2 run nav2_map_server map_saver_cli -f my_cool_map
```
This will generate `my_cool_map.yaml` and `my_cool_map.pgm`.

## 🛠 Advanced Configuration
You can adjust the resolution (default: 5cm) and other parameters in:
`src/xpi_navigation/config/mapper_params_online_async.yaml`
