set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
sh -c 'echo "deb http://lcas.lincoln.ac.uk/repos/release trusty main" > /etc/apt/sources.list.d/lincoln_server.list'
wget http://lcas.lincoln.ac.uk/repos/public.key -O - | apt-key add -
apt-get update

sh -c 'echo "deb http://rwiki.acin.tuwien.ac.at/apt/v4r-release trusty main" > /etc/apt/sources.list.d/rwiki_server.list'
apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key 943EB54F
apt-get update

mkdir -p /etc/ros/rosdep/mapping/
cp squirrel_perception/rosdep/private.yaml /etc/ros/rosdep/mapping/private_perception.yaml
cp squirrel_perception/rosdep/31-squirrel_perception.list /etc/ros/rosdep/sources.list.d/31-squirrel_perception.list
rosdep update
