#!/bin/bash

usage="$(basename "$0") [-h] [-n #] -- script to start vehicle sim and aclswarm nodes

where:
    -h  show this help text
    -n  number of vehicles to simulate"

if [ "$#" -le 1 ]; then
    echo "$usage"
    exit 1
fi

while getopts ':hn:' option; do
  case "$option" in
    h) echo "$usage"
       exit
       ;;
    n) num=$OPTARG
       ;;
    :) printf "missing argument for -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
   \?) printf "illegal option: -%s\n" "$OPTARG" >&2
       echo "$usage" >&2
       exit 1
       ;;
  esac
done

echo "Starting vehicle sims"

tmux -2 new-session -d -s 'sims'
# tmux set pan-boarder-status top

tmux split-window -h
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
if (( $num > 4 ))
then
	tmux split-window -v
	tmux split-window -v
	tmux split-window -v
	tmux select-pane -t 5
	tmux split-window -v
	if (( $num > 9 ))
	then
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux select-pane -t 10
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux split-window -h
	fi
fi
tmux select-layout tiled

for _sim in $(eval echo {01..$num}); do
	_pane=`echo "$_sim - 1" | bc`
	echo "Starting sim $_sim"
	x=`echo "($_pane % 5) * 1.5 - 4" | bc`
	y=`echo "($_pane / 5) * 1.5" | bc`
	tmux send-keys -t ${_pane} "roslaunch quad_sim quad_sim.launch num:=$_sim x:=$x y:=$y" C-m
	sleep 1.0
done

echo "Starting aclswarm nodes"

tmux -2 new-session -d -s 'aclswarm'
# tmux set pan-boarder-status top

tmux split-window -h
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
if (( $num > 4 ))
then
	tmux split-window -v
	tmux split-window -v
	tmux split-window -v
	tmux select-pane -t 5
	tmux split-window -v
	if (( $num > 9 ))
	then
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux select-pane -t 10
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux split-window -v
		tmux split-window -h
	fi
fi
tmux select-layout tiled

for _sim in $(eval echo {01..$num}); do
	_pane=`echo "$_sim - 1" | bc`
	echo "Starting aclswarm $_sim"
	_veh="SQ$_sim"
	_veh+="s"
	tmux send-keys -t ${_pane} "roslaunch aclswarm dist_form_ctrl.launch room:=vicon veh:=$_veh leader:=0" C-m
	sleep 1.0
done

tmux a -t "aclswarm"
