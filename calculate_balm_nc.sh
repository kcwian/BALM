#!/bin/bash
mkdir -p output_nc
for i in $(seq 1 1)
do
  roslaunch balm2 stairs.launch voxel_size:=$i
done
roslaunch balm2 stairs.launch voxel_size:=0.2

for i in $(seq 1 1)
do
  roslaunch balm2 cloister.launch voxel_size:=$i
done

for i in $(seq 1 1)
do
  roslaunch balm2 maths_easy.launch voxel_size:=$i
done

for i in $(seq 1 1)
do
  roslaunch balm2 underground_easy.launch voxel_size:=$i
done

for i in $(seq 1 1)
do
  roslaunch balm2 quad_easy.launch voxel_size:=$i
done
