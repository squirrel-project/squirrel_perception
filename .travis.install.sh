set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
apt-get install apt-transport-https
sh -c 'echo "deb [arch=amd64] https://rwiki.acin.tuwien.ac.at/apt/v4r-release trusty main" > /etc/apt/sources.list.d/squirrel_server.list'
apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 943EB54F
apt-get update

mkdir -p /etc/ros/rosdep/mapping/
cp squirrel_perception/rosdep/private.yaml /etc/ros/rosdep/mapping/private_perception.yaml
cp squirrel_perception/rosdep/31-squirrel_perception.list /etc/ros/rosdep/sources.list.d/31-squirrel_perception.list
rosdep update
