#!/bin/bash
set -e

echo ------------------------------- Stairs \n\n\n
roslaunch balm2 stairs.launch
echo ------------------------------- Clositer \n\n\n
roslaunch balm2 cloister.launch
echo ------------------------------- Maths easy \n\n\n
roslaunch balm2 maths_easy.launch
echo ------------------------------- Underground easy \n\n\n
roslaunch balm2 underground_easy.launch
echo ------------------------------- Quad easy \n\n\n
roslaunch balm2 quad_easy.launch
#roslaunch balm2 park.launch
