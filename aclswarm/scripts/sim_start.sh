#!/bin/bash

function make_tmux_session {
  cmd="new-session -d -s $1"
  for ((i=1; i<$2; i++)); do
    cmd="$cmd ; split-window ; select-layout tiled"
  done
  # add any extra commands
  cmd="$cmd ; ${3:-}"
  tmux -2 $cmd
}

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

#
# quad sim
#

echo "Starting vehicle sims"
make_tmux_session sims $num

for _sim in $(eval echo {01..$num}); do
	_pane=`echo "$_sim - 1" | bc`
	echo "Starting sim $_sim"
	x=`echo "($_pane % 5) * 1.5 - 4" | bc`
	y=`echo "($_pane / 5) * 1.5" | bc`
  tmux send-keys -t sims:0.${_pane} "roslaunch quad_sim quad_sim.launch num:=$_sim x:=$x y:=$y" C-m
done

# send commands to all panes
tmux set-window-option -t sims:0 synchronize-panes on

#
# aclswarm stack
#

echo "Starting aclswarm nodes"
make_tmux_session aclswarm $num

for _sim in $(eval echo {01..$num}); do
	_pane=`echo "$_sim - 1" | bc`
	echo "Starting aclswarm $_sim"
	_veh="SQ${_sim}s"
	tmux send-keys -t aclswarm:0.${_pane} "roslaunch aclswarm start.launch veh:=$_veh leader:=0" C-m
done

# send commands to all panes
tmux set-window-option -t aclswarm:0 synchronize-panes on

#
# attach to aclswarm stack session
#

tmux attach-session -t aclswarm

