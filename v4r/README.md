This is the git mirror of ACIN's v4r library.

*NOTE: If you are building on ubuntu 12.04*
Ubuntu 12.04 has PCL 1.7.0. v4r was built against PCL 1.7.1. You need to replace two files to use v4r on 12.04:
```
/usr/include/pcl-1.7/pcl/segmentation/supervoxel_clustering.h
/usr/include/pcl-1.7/pcl/registration/correspondence_rejection_sample_consensus.h
```
You find these files in the tmp folder. Copy (as root) over the original files, after backing these up.
