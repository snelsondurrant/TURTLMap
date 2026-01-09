#!/bin/bash
docker exec -it turtlmap-ct bash -c "source /root/catkin_ws/devel/setup.bash && roslaunch turtlmap run_log2.launch"
