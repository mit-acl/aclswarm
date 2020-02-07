#!/bin/bash

# Get a list of the current vehicles in use
vehstr=$(rosparam get "/vehs" | tr -d "\n" | cut -d "[" -f 2 | cut -d "]" -f 1)
IFS=", " read -r -a vehs <<< "$vehstr"

# a list of topics to record. Topics without a leading
# slash are assumed to exist under the vehicle namespaces.
declare -a raw_topics=(\
            "state" \
            "distcmd" \
            "goal" \
            "safety/status" \
            "cbaabid" \
            "assignment" \
            "world" \
            "/formation" \
            "/viz_safe_cmd" \
            "/viz_dist_cmd" \
            "/viz_central_alignment" \
            "/viz_mesh" \
            "/Tag01/world" \
            "/Tag02/world"
            )

# A place to put the parsed topics to be recorded
TOPICS=""

for n in "${!vehs[@]}"; do
    for t in "${raw_topics[@]}"; do
        if [[ $t == /* ]]; then
            if [[ "$n" == 0 ]]; then
                # add to list without alteration
                TOPICS+=" $t"
            fi
        else
            TOPICS+=" /${vehs[$n]}/$t"
        fi
    done
done

rosbag record $TOPICS "$@"
