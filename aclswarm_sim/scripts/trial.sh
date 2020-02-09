#!/bin/bash

# Get path to the directory of this file, no matter where it is sourced from
MYPATH=$(dirname ${BASH_SOURCE[0]})

# n=6   # number of agents --- deduced from formation
w=20    # width of initialization area
h=20    # height of initialization area
r=0.75  # buffer radius btwn initializations

#
# parse argument
#

trialname=$1
trialnumber=$2
formation=$3
interactive=$4
use_leader=$5

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

if [[ "$formation" == simform* ]]; then
  numAgents=$(echo "${formation//[!0-9]/}")
  formation="simform"

  # this generates a formation and sets the appropriate rosparam
  rosrun aclswarm_sim generate_random_formation.py -l 20 -w 20 -h 5 "$numAgents"
fi

# cheat: the launch file also does this, but there is a dependency loop unless we do this
rosparam load "$(rospack find aclswarm)/param/formations.yaml" operator

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

# Launch the operator with any necessary flags
extras=""
if [ $interactive == "false" ]; then
  extras="$extras headless:=false"
fi
if [ $use_leader == "true" ]; then
  extras="$extras leader:=SQ01s"
fi
roslaunch aclswarm operator.launch "formations:=$formation" load_vehicles:=false $extras &

rviz >/dev/null 2>&1 &

#
# Set room bounds for simulation
#

rosparam set /room_bounds "{x_min: -100, x_max: 100, y_min: -100, y_max: 100, z_min: 0, z_max: 30}"

#
# Start swarm simulations
#

rosrun aclswarm_sim start.sh -n $n -r "${w}x${h}x${r}" -x

sleep 5 # wait for system bring up

#
# Simulation trial
#

# rosrun aclswarm_sim bag_record.sh -O "$formation$trialname$trialnumber" __name:=bagrecorder >/dev/null 2>&1 &

if [ $interactive == "false" ]; then
  rosrun aclswarm_sim supervisor.py
else
  sleep 3 # wait for snapstack sim to initialize
  echo -e "\e[97;1mSimulation initialized. You may now press '\e[32;1mSTART\e[97;1m'.\e[0m"

  # get pid of controls rqt
  rqtpid=$(pgrep -f rqt_gui)
  echo -e "\e[97;1mOnce rqt_gui ($rqtpid) is quit, simulation will clean up.\e[0m"
  tail --pid=$rqtpid -f /dev/null # kind of a hack
fi

# rosnode kill /bagrecorder
# sleep 1

#
# Cleanup
#

# be a good computer citizen!!1!
tmux kill-server
pkill -x -9 rviz
pkill -f -9 operator.py
pkill -f -9 viz_commands.py
pkill -f -9 supervisor.py
pkill -x -9 roslaunch
pkill -x -9 snap_sim
pkill -x -9 snap
pkill -f -9 outer_loop
pkill -x -9 localization
pkill -x -9 coordination
pkill -x -9 safety
pkill -f -9 static_transform_publisher
pkill -f -9 throttle
pkill -x -9 rosout
pkill -x -9 roscore
pkill -x -9 rosmaster
