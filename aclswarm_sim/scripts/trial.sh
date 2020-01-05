#!/bin/bash

# Get path to the directory of this file, no matter where it is sourced from
MYPATH=$(dirname ${BASH_SOURCE[0]})

# n=6   # number of agents --- deduced from formation
w=20    # width of initialization area
h=20    # height of initialization area
r=0.75  # buffer radius btwn initializations

formparam="$MYPATH/../param/formations.yaml"
formation=$1

if [ -z "$formation" ]; then
  echo
  echo "A formation group is required"
  echo
  exit 1
fi

#
# Start ROS
#

# check that a roscore is NOT running
if pgrep -x roscore > /dev/null ; then
  echo
  echo "A roscore is already running. Shutdown roscore and try again."
  echo
  exit 2
fi

roscore >/dev/null 2>&1 &
sleep 1 # wait for roscore to initialize

#
# Start Operator
#

# set formation
rosparam load "$formparam" operator
rosparam set /operator/formation_group "$formation"

# retrieve how many agents are needed for this formation
n=$(rosparam get "/operator/${formation}/agents")

# set vehicle list as a rosparam (SQ00s)
for i in $(seq -f "%02g" 1 $n); do
  veh="SQ${i}s"
  if [ "$i" -eq "01" ]; then
    tmp="$veh"
  else
    tmp="$tmp, $veh"
  fi
done
rosparam set /vehs "[$tmp]"

roslaunch aclswarm_sim headless_operator.launch >/dev/null 2>&1 &

rviz &

#
# Start swarm simulations
#

cd $MYPATH
bash start.sh -n $n -r "${w}x${h}x${r}" -x

sleep 3 # wait for system bring up

#
# Simulation trial
#

rosrun aclswarm_sim supervisor.py

#
# Cleanup
#

# be a good computer citizen!!1!
tmux kill-server
pkill -x rviz
pkill -x roslaunch
pkill -x roscore
pkill -x -9 vicon_relay
pkill -x -9 quad_sim
pkill -x -9 quad_controller
pkill -x -9 localization
pkill -x -9 coordination
pkill -x -9 safety
pkill -x -9 rosmaster
pkill -x -9 rosout