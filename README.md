# Team 34: SICK LiDAR 3 Retail Assistant Robot
Fall 2025 - Spring 2026 ECEN 403/404 Capstone Project  \
  \
Create an autonomous retail assistant robot which is capable of detecting & capturing location of stocking gaps & provide location assistance to customers.  

Each branch holds various project features:
- **main:** the most up-to-date, fully functional version of the 'interface' branch, hosted via Render, and connected to a Firebase backend as defined by local variables on startup
- **interface:** the working branch for customer/staff user interfaces, connects to the Firebase backend to view captured photographs, photograph notifications, store map, store inventory, and set rover navigation points
- **stm32hal:** the C++ STM32 HAL code flashed onto the custom designed STM32G474RC PCB using an ST-Link device jumper wired to the serial data ports, features libraries for imu data, motor control, pid control loop, stm32-to-pi communication
- **SLAM_NAV_Gazebo_classic:** ROS 2 Gazebo Classic SLAM & AMCL navigation code, pi-to-stm32 communication, LiDAR gap detection algorithm, and map/camera database uploader, all hosted on the connected Raspberry Pi
