#!/bin/bash
#if ./start.sh uav 4  $1=uav;$2=4
python3 leader.py $1 $2 &  
#python3 avoid.py $1 $2 vel &
uav_id=1           #jump leader
while(( $uav_id< $2 )) 
do
    python3 follower.py $1 $uav_id $2 &
    let "uav_id++"
done
