#!/bin/bash

# Get path to the directory of this file, no matter where it is sourced from
MYPATH=$(dirname ${BASH_SOURCE[0]})

usage="$(basename "$0") [-h] -m <#> -f <formation> -n <name> -- perform m simulation trials

where:
    -h                    show this help text
    -m <#>                number of trials to simulate
    -f <formation_group>  which formation group to use
    -n <name>             name of trials for logging"

if [ "$#" -le 2 ]; then
    echo "$usage"
    exit 1
fi

while getopts 'hm:f:n:' option; do
  case "$option" in
    h)
      echo "$usage"
      exit
      ;;
    m)
      m=$OPTARG
      ;;
    f)
      formation_group=$OPTARG
      ;;
    n)
      name=${OPTARG}_
      ;;
   \?)
      printf "illegal option: -%s\n" "$OPTARG" >&2
      echo "$usage" >&2
      exit 1
      ;;
  esac
done

if [[ -z $name ]]; then
  name=
fi

for i in $(seq 1 $m); do

  echo
  echo -e "\e[34;1mStarting Trial $i of $m\e[0m"
  echo

  # zero pad based on total number of trials
  printf -v j "%0${#m}d" $i

  bash $MYPATH/trial.sh $formation_group "${name}$j"

  sleep 1
done