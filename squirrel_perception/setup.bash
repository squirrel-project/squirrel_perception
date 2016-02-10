#!/usr/bin/env bash
source setup.sh && source `pwd`/../../../devel/setup.bash

if [ $? -ne 0 ]; then
  echo "Unable to source setup.bash in your catkin workspace. Please run this script from within the squirrel_perception directory."
  exit 1
fi

generate_path
check_file
if [ $? -ne 0 ]; then
  echo "Error occured. Check output"
  exit 1
fi
check_repo
if [ $? -ne 0 ]; then
  echo "Error occured. Check output"
  exit 1
fi
echo "Done"
exit 0
