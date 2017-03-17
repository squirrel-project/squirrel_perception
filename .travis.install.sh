set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
sh -c 'echo "deb http://153.97.4.193/building/ubuntu trusty main" > /etc/apt/sources.list.d/squirrel_server.list'
wget http://153.97.4.193/building/public.key -O - | apt-key add -
apt-get update

sh -c 'echo "deb http://lcas.lincoln.ac.uk/repos/release trusty main" > /etc/apt/sources.list.d/lincoln_server.list'
wget http://lcas.lincoln.ac.uk/repos/public.key -O - | apt-key add -
apt-get update

mkdir -p /etc/ros/rosdep/mapping/
cp squirrel_perception/rosdep/private.yaml /etc/ros/rosdep/mapping/private_perception.yaml
cp squirrel_perception/rosdep/31-squirrel_perception.list /etc/ros/rosdep/sources.list.d/31-squirrel_perception.list
rosdep update

