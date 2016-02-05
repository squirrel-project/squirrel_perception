#!/usr/bin/env zsh
source setup.sh
source ~/.zshrc
# The call to squirrel alias is only needed on bajo's computer
#squirrel

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
