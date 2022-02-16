# LaserSLAM based on C#.

This is 2D LaserSLAM program based on C# language.
There's still room for improvement.

YDLidarG2 is used.

![image](https://user-images.githubusercontent.com/37968684/154209230-10ef51bb-6a88-4a1b-9bf1-6a28443d3f6d.png)


# Source code list

1. Program.cs (main)
2. SLAM_utils.cs (SLAM utility function class)
### SLAM_utils function list
 - create_BWmap : Convert local map to hamming distance map
 - distanceBW : hamming distance 
 - extract_sgrid : extract small grid from local map
 - cal_idx : calculate index from small grid
 - ExtractLocalMap : extract local map from map
 - transform
 - pol2cart : convert polar coordinates to cartesian coordinates
 - Dgetr
 - deg2rad : convert degree to radian
 - rad2deg(not used)
 - diffpos : difference between current position and previous position
 - min_max_func : get min(x, y), max(x, y) from map


**Reference :**

1. YDLidarSharp github : https://github.com/Denny9700/YDLidarSharp.git

2. LaserSLAM github : https://github.com/meyiao/LaserSLAM.git (W.Hess, D.Kohler, H.Rapp and D.Andor. Real-Time Loop Closure in 2D LIDAR SLAM. ICRA, 2016)
