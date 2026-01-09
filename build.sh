#!/bin/bash
docker exec -it turtlmap-ct bash -c "cd /root/catkin_ws && source devel/setup.bash && catkin build turtlmap --no-deps"
