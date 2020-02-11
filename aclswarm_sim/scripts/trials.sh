#!/bin/bash

# Get path to the directory of this file, no matter where it is sourced from
MYPATH=$(dirname ${BASH_SOURCE[0]})

_trap() {
  kill -INT "$trialpid"
  wait "$trialpid"
  exit 1
}

trap _trap INT

usage="$(basename "$0") [-h] -m <#> -f <formation> -n <name> -i -l -- perform m simulation trials

where:
    -h                    show this help text
    -m <#>                number of trials to simulate
    -s                    Monte Carlo mode---use trial num as randseed
    -f <formation_group>  which formation group to use
    -n <name>             name of trials for logging
    -i                    interactive operator
    -l                    use SQ01s as a leader"

if [ "$#" -le 2 ]; then
    echo "$usage"
    exit 1
fi

use_leader="false"
interactive="false"
montecarlo="false"

while getopts 'hm:sf:n:il' option; do
  case "$option" in
    h)
      echo "$usage"
      exit
      ;;
    m)
      m=$OPTARG
      ;;
    s)
      montecarlo="true"
      ;;
    f)
      formation_group=$OPTARG
      ;;
    n)
      name=${OPTARG}
      ;;
    i)
      interactive="true"
      ;;
    l)
      use_leader="true"
      ;;
   \?)
      printf "illegal option: -%s\n" "$OPTARG" >&2
      echo "$usage" >&2
      exit 1
      ;;
  esac
done

if [[ -z $name ]]; then
  name=aclswarm_trials
fi

if [ "$interactive" == "true" ]; then
  m=1
fi

if [ -f "$name.csv" ]; then
  mv "$name.csv" "$name.csv.bak"
fi

for i in $(seq 1 $m); do

  echo
  echo -e "\e[34;1mStarting Trial $i of $m\e[0m"
  echo

  # zero pad based on total number of trials
  printf -v j "%0${#m}d" $i

  rosrun aclswarm_sim trial.sh "$name" "$j" "$formation_group" "$interactive" "$use_leader" "$montecarlo" &
  trialpid=$!
  wait $trialpid

  sleep 1
done