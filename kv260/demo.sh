#!/bin/bash

SESSION="rail_detection"

if [ "$(whoami)" == "root" ]; then
    tmux new-session -d -s $SESSION

    tmux split-window -h
    # tmux split-window -h

    # tmux send-keys -t 0 "python3 /home/ubuntu/rail_detection/uav_rail_tracking/video_test.py" C-m
    tmux send-keys -t 0 "python3 /home/ubuntu/kv260/cam_test.py" C-m

    tmux send-keys -t 1 "/home/ubuntu/kv260/yolov5n_v4.elf"  

    # tmux send-key -t 2 "python3 /home/ubuntu/rail_detection/uav_rail_tracking/drone_info_server.py" 


    tmux -2 attach-session -t $SESSION
else
    echo "sudo please!!!"
fi


