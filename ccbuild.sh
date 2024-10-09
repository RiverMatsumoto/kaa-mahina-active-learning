rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --allow-overriding smacc2 smacc2_msgs --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 1
