#!/bin/bash

# Get path to the directory of this file, no matter where it is sourced from
MYPATH=$(dirname ${BASH_SOURCE[0]})

usage="$(basename "$0") [-h] -m <#> -f <formation> -- perform m simulation trials

where:
    -h                    show this help text
    -m <#>                number of trials to simulate
    -f <formation_group>  which formation group to use"

if [ "$#" -le 2 ]; then
    echo "$usage"
    exit 1
fi

while getopts 'hm:f:' option; do
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
   \?)
      printf "illegal option: -%s\n" "$OPTARG" >&2
      echo "$usage" >&2
      exit 1
      ;;
  esac
done

for i in $(seq 1 $m); do

  echo
  echo -e "\e[34;1mStarting Trial $i of $m\e[0m"
  echo

  bash $MYPATH/trial.sh $formation_group

  sleep 1
done