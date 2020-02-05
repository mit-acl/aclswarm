#!/bin/bash

VEH=$1
NUM=$2
QUAD=$VEH$NUM
SESSION=remote_$QUAD

if [ -z "$1" ]
    then
        echo "No vehicle selected"
        echo "Example use: ./remote_start.sh SQ 02"
        exit
    else
        echo "Starting vehicle $QUAD"
fi

# check that an ssh connection can even be made
ssh -q root@$QUAD.local exit
if [ $? -ne 0 ]; then
    echo -e "\033[0;31mNo connection to $QUAD\033[0m"
    exit
fi

cmd="new-session -d -s $SESSION"
cmd="$cmd ; split-window -h"
cmd="$cmd ; split-window -v"
cmd="$cmd ; select-pane -t $SESSION:0.0"
cmd="$cmd ; split-window -v"
cmd="$cmd ; select-pane -t $SESSION:0.2"
cmd="$cmd ; split-window -v"
cmd="$cmd ; select-pane -t $SESSION:0.1"
cmd="$cmd ; split-window -v"
cmd="$cmd ; select-pane -t $SESSION:0.5"
cmd="$cmd ; split-window -v"

tmux -2 $cmd

for _pane in $(tmux list-pane -F '#P'); do
    tmux send-keys -t $SESSION:0.${_pane} "ssh root@$QUAD.local" C-m
    sleep 1
done

tmux send-keys -t $SESSION:0.4 "imu_app -s 2" C-m
tmux send-keys -t $SESSION:0.5 "roslaunch snap snap.launch" C-m
tmux send-keys -t $SESSION:0.0 "roslaunch outer_loop cntrl.launch" C-m
tmux send-keys -t $SESSION:0.3 "roslaunch snap esc.launch" C-m
tmux send-keys -t $SESSION:0.1 "roslaunch aclswarm start.launch veh:=$VEH$NUM"
tmux send-keys -t $SESSION:0.2 "roslaunch vislam vislam.launch initLnDepth:=-2.617" # on stand
#tmux send-keys -t $SESSION:0.6 "rosrun aclswarm bagrecord.sh -o $VEH$NUM"

tmux select-pane -t $SESSION:0.1
tmux -2 attach-session -t $SESSION
