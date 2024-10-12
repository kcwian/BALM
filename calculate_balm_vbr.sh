#!/bin/bash
#set -e
mkdir -p output_vbr
for i in $(seq 1 1)
do
  roslaunch balm2 campus.launch voxel_size:=$i
done
for i in $(seq 1 1)
do
  roslaunch balm2 ciampino.launch voxel_size:=$i
done
for i in $(seq 1 1)
do
  roslaunch balm2 colloseo.launch voxel_size:=$i
done
for i in $(seq 1 1)
do
  roslaunch balm2 diag.launch voxel_size:=$i
done
for i in $(seq 1 1)
do
  roslaunch balm2 pincio.launch voxel_size:=$i
done
for i in $(seq 1 1)
do
  roslaunch balm2 spagna.launch voxel_size:=$i
done


