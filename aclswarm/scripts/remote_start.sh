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

tmux -2 new-session -d -s $SESSION
# tmux set pan-boarder-status top

tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 5
tmux split-window -v

for _pane in $(tmux list-pane -F '#P'); do
	# ssh into vehicle using env variables from cfg file
	tmux send-keys -t ${_pane} "ssh root@$QUAD.local" C-m
	sleep 1
done

tmux send-keys -t 4 "imu_app -s 2" C-m
tmux send-keys -t 5 "roslaunch snap snap.launch veh:=$VEH num:=$NUM" C-m
tmux send-keys -t 0 "roslaunch system_launch quad.launch veh:=$VEH num:=$NUM" C-m
tmux send-keys -t 3 "rosrun snap esc_interface_node __ns:=$VEH$NUM" C-m
tmux send-keys -t 1 "roslaunch aclswarm start.launch veh:=$VEH$NUM"
#tmux send-keys -t 2 "roslaunch vislam vislam.launch"
#tmux send-keys -t 6 "rosrun aclswarm bagrecord.sh -o $VEH$NUM"

tmux select-pane -t 1
tmux -2 attach-session -t $SESSION
