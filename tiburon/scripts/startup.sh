#!/bin/bash
alias connectauv='ssh auv-nitr@192.168.61.154'
source /home/sazzy_techgirl/Documents/tiburon-master/devel/setup.bash
roscd #devel folder
cd .. #Now we are inside tiburon-master package
catkin_make #optional
rosrun tiburon 
