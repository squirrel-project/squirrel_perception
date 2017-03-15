set -e
set -v

echo "BEFORE INSTALL IS RUNNING"
sudo sh -c 'echo "deb http://153.97.4.193/building/ubuntu trusty main" > /etc/apt/sources.list'
wget http://153.97.4.193/building/public.key -O - | sudo apt-key add -
