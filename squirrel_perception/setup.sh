#!/usr/bin/env bash
FILE="/etc/ros/rosdep/sources.list.d/31-squirrel_perception.list"
REPO="http://153.97.4.193/building/ubuntu/"
REPO_STRANDS="http://lcas.lincoln.ac.uk/repos/release"
APTSOURCES="/etc/apt/sources.list"

generate_path () {
  YAML=""
  roscd squirrel_perception
  if [ $? -ne 0 ]; then
    echo "unable to execute roscd squirrel_perception. Please check and re-run"
    return 1 
  fi 
  YAML="yaml file://"`pwd`"/rosdep/private.yaml"
  return 0 
}

write_file () {
  sudo mkdir -p /etc/ros/rosdep/mapping/
  sudo cp rosdep/private.yaml /etc/ros/rosdep/mapping/private_perception.yaml
  sudo cp rosdep/31-squirrel_perception.list $FILE
}

check_file () {
  # Check if we already setup the rosdep sources.list
  if grep -Fxq "$YAML" $FILE
  then
      # code if found
      echo "Nothing to do here"
      return 0
  else
      # code if not found
      echo "not in there yet"
      write_file
      rosdep update
      return 0
  fi
}

add_repo () {
  curl -s http://153.97.4.193/building/public.key | sudo apt-key add -
  curl -s http://lcas.lincoln.ac.uk/repos/public.key | sudo apt-key add -
  sudo apt-add-repository $REPO
  sudo apt-add-repository $REPO_STRANDS
  sudo apt-get update
}

check_repo () {
  # Check if we already setup the rosdep sources.list
  if grep -q "$REPO" $APTSOURCES
  then
      # code if found
      echo "STRANDS repo is already included"
      return 0
  else
      # code if not found
      echo "repo is not in there yet"
      if [ $? -ne 0 ]; then
        echo "Exit with error"
        return 1 
      fi 
      add_repo
      return 0
  fi
}
