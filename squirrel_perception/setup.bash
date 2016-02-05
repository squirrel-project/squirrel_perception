#!/usr/bin/env bash
source setup.sh
source ~/.bashrc

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
