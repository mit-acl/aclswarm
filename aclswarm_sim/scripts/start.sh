#!/bin/bash
# Starts a simulation trial.
# 
# Remember: However long your .bashrc takes will be multiplied by n.
# For best results, make your .bashrc as lean as possible and source
# the relevant aclswarm catkin workspace in the .bashrc.
# 
# TODO: tmux may not be the best for larger simulations (> 50). Consider
# managing each process without tmux.

function draw_uniform {
  # generates a random float in [a b)
  # n.b.: a, b must be ints
  a=$1
  b=$2
  echo "$((a+RANDOM%(b-a))).$((RANDOM%999))"
}

function draw_nonoverlapping_circles {
  # Samples n non-overlapping circles from a uniform square with radius r
  n=$1
  r=$2
  w=$3
  h=$4

  # range of box, centered at the origin (int for draw_uniform)
  w1=$(echo "scale=0;$w / 2.0 * -1" | bc)
  w2=$(echo "scale=0;$w / 2.0" | bc)
  h1=$(echo "scale=0;$h / 2.0 * -1" | bc)
  h2=$(echo "scale=0;$h / 2.0" | bc)

  circles_x=()
  circles_y=()

  while [ ${#circles_x[@]} -lt $n ]; do
    # Sample a new circle
    x=$(draw_uniform $w1 $w2)
    y=$(draw_uniform $h1 $h2)

    # Compare where the new circle is w.r.t the old circles
    overlapped=false
    for i in "${!circles_x[@]}"; do
      # calculate distance from center of new circle to this circle
      dx=$(echo "$x - ${circles_x[$i]}" | bc)
      dy=$(echo "$y - ${circles_y[$i]}" | bc)
      d=$(echo "scale=4;sqrt($dx * $dx + $dy * $dy)" | bc)

      # skip if it is too close to previously sampled circles
      if (( $(echo "$d < 2 * $r" | bc -l)  )); then
        overlapped=true
        break
      fi
    done

    # If we've made it here, this circle does not overlap
    if [ "$overlapped" = false ]; then
      circles_x+=($x)
      circles_y+=($y)
    fi
  done
}

function make_tmux_session {
  cmd="new-session -d -s $1 -x 300 -y 300"
  for ((i=1; i<$2; i++)); do
    cmd="$cmd ; split-window ; select-layout tiled"
  done
  # add any extra commands
  cmd="$cmd ; ${3:-}"
  tmux -2 $cmd
}

usage="$(basename "$0") [-h] [-n #] -- script to start vehicle sim and aclswarm nodes

where:
    -h              show this help text
    -n              number of vehicles to simulate
    -r <w>x<h>x<r>  random initialization in [-w/2 w/2]x[-h/2 h/2] with radius r
    -x              headless---do not attach to tmux at end"

if [ "$#" -le 1 ]; then
    echo "$usage"
    exit 1
fi

while getopts 'hn:r:x' option; do
  case "$option" in
    h)
      echo "$usage"
      exit
      ;;
    n)
      num=$OPTARG
      ;;
    r)
      w=$(echo $OPTARG | cut -d'x' -f1)
      h=$(echo $OPTARG | cut -d'x' -f2)
      r=$(echo $OPTARG | cut -d'x' -f3)
      rtxt=" (randomly initialized in $w x $h square; buffer radius $r)"
      ;;
    x)
      headless=true
      ;;
   \?)
      printf "illegal option: -%s\n" "$OPTARG" >&2
      echo "$usage" >&2
      exit 1
      ;;
  esac
done

#
# quad sim
#

echo -e "\e[33;1mStarting Simulation\e[0m"
echo -e $rtxt
make_tmux_session sims $num

# generate initialization points if needed
if [ ! -z "$rtxt" ]; then
  draw_nonoverlapping_circles $num $r $w $h
  # echo "X: ${circles_x[@]}"
  # echo "Y: ${circles_y[@]}"
fi

for _sim in $(eval echo {01..$num}); do
  _pane=`echo "$_sim - 1" | bc`
  if [ -z "$rtxt" ]; then
    x=$(echo "($_pane % 5) * 1.5 - 4" | bc)
    y=$(echo "($_pane / 5) * 1.5" | bc)
  else
    x=${circles_x[$_pane]}
    y=${circles_y[$_pane]}
  fi
  tmux send-keys -t sims:0.${_pane} "roslaunch quad_sim quad_sim.launch num:=$_sim x:=$x y:=$y" C-m
done

# send commands to all panes
tmux set-window-option -t sims:0 synchronize-panes on

#
# aclswarm stack
#

make_tmux_session aclswarm $num

for _sim in $(eval echo {01..$num}); do
  _pane=`echo "$_sim - 1" | bc`
  _veh="SQ${_sim}s"
  tmux send-keys -t aclswarm:0.${_pane} "roslaunch aclswarm start.launch veh:=$_veh leader:=0" C-m
done

# send commands to all panes
tmux set-window-option -t aclswarm:0 synchronize-panes on

#
# attach to aclswarm stack session
#

if [ -z $headless ]; then
  tmux attach-session -t aclswarm
fi
