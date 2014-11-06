# ACIN V4R mirror

This is the git mirror of ACIN's v4r library for STRANDS.

It uses `git svn` to pull in the latest v4r strands branch from ACIN's svn server:
https://repo.acin.tuwien.ac.at/v4r/stable/strandsv4r

At the moment, these frameworks need a particular PCL version to run. In order to avoid conflicts with other pcl versions (i.e. the built-in ros-pcl version), please follow these steps:

1.) install required system packages:

    sudo apt-get install libsuitesparse-dev
    sudo apt-get install libglm-dev
    libglew1.5-dev

2.) check out PCL as follows into a local directory:

    git clone https://github.com/arbeitor/pcl.git ~/pcl_v4r_fork --branch reconstruction_workshop

3.)  build PCL

    mkdir ~/pcl_v4r_fork/build
    cd ~/pcl_v4r_fork/build
    cmake ..
    make -j4

**IMPORTANT: DO NOT INSTALL PCL to avoid conflicts with other versions!**

4.) set V4R directory in cmake (Note: this will be fixed)

    cd YOUR_CATKIN_WS
    cd build
    ccmake .

Then set V4R_DIR to YOUR_CATKIN_WS/src/v4r

5.) tell catkin to use _pcl\_v4r\_fork_ for packages dependent on this particular pcl version:

    catkin_make -DV4R_PCL_DIR=~/pcl_v4r_fork/build


## For users

Normal users of this repository just run `git pull`.
Normal uses should never need to push to this repository.


## For maintainers

### Set up the svn git bridge
Run all this NOT from within any of your regular git work spaces.

    git svn clone https://repo.acin.tuwien.ac.at/v4r/stable/strandsv4r
    cd strandsv4r
    git checkout -b hydro-devel
    git remote add origin https://github.com/strands-project/v4r.git
    git push -u origin hydro-devel

### Update
Whenver the svn changed, you have to move these changes to git.

    cd strandsv4r
    git svn rebase
    git push

**It is recommended that usually bugfixes etc for V4R are *not* committed directly to this repository, but rather are fixed in the upstream SVN repository**

---

This repository has been set up following (some of) the instructions at [http://git-scm.com/book/en/Git-and-Other-Systems-Git-and-Subversion]