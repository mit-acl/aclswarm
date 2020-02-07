#!/bin/bash

# Get a list of the current vehicles in use
vehstr=$(rosparam get "/vehs" | tr -d "\n" | cut -d "[" -f 2 | cut -d "]" -f 1)
IFS=", " read -r -a vehs <<< "$vehstr"

for n in "${!vehs[@]}"; do
    # register vislam with extpose
    rosservice call /${vehs[$n]}/pose_selector/sample
done

sleep 5

for n in "${!vehs[@]}"; do
    # select vislam
    rosservice call /${vehs[$n]}/pose_selector/select "data: true"
done

