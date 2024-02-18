#!/bin/bash

SESSION="rail_detection"
cam_script="/home/ubuntu/uav_rail_detection/video_test.py"

if [ "$(whoami)" == "root" ]; then
    tmux new-session -d -s $SESSION
    tmux split-window -h
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -v

    tmux send-keys -t 0 "python3 $cam_script " C-m
    
    # tmux send-keys -t 1 "echo \"sleep for 15 second...\"" C-m
    # tmux send-keys -t 1 "sleep 15" C-m
    tmux send-keys -t 1 "/home/ubuntu/kv260/yolov5n_v3.elf" 

    # tmux send-keys -t 2 "echo \"sleep for 18 second...\"" C-m
    # tmux send-keys -t 2 "sleep 18" C-m
    # tmux send-keys -t 2 "python3 /home/ubuntu/drone/main_demo.py > ./log/drone_command_log.txt" 
    tmux send-keys -t 2 "python3 /home/ubuntu/drone/drone_info_write.py" C-m

    # tmux send-keys -t 3 "echo \"sleep for 25 second...\"" C-m
    # tmux send-keys -t 3 "sleep 25" C-m
    tmux send-key -t 3 "python3 /home/ubuntu/uav_rail_detection/server.py" C-m

    tmux -2 attach-session -t $SESSION
else
    echo "sudo please!!!"
fi
 
