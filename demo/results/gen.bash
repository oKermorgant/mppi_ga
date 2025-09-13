#!/usr/bin/env bash

robot="bicycle"
# robot="rov"
robot="pend"
# robot="boat"
# multi=0

for multi in 0;
do
    # for robot in bicycle rov;
    # do
#     for pop in 100 200 300 400 500;
    for pop in 10 30 50 70 90;
    do
        for bf in 0;
        do
            cmd="../../build/demo/$robot hor 11 plot 0  pop $pop multi $multi bf $bf"
            $cmd sub 1
            for z in 0 1;
            do
                for i in 2 3 4 5;
                do
                    $cmd sub $i use_zoh $z
                done
            done
        done
    done
done

./summary.py $robot
