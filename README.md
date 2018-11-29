# PointCloudScissor
A useful tool to cut a set of point cloud into two parts with a designed IoU (overlapping)

## Now available on Windows.

### PCL 1.8 needed
### Compile with VS2013


## How to use 

### 1.Input the point cloud to be cut in PCD format.

### 2.Input the designed IoU (overlapping ratio) 

IoU (ranging from 0 to 1) is calculated as the number of points in intersection area over the point number of the original point cloud.

For example, for a cut into two non-overlapping segmentations, IoU should be set as 0.

### 3. Select the cut direction 

Select from 1.X, 2.Y, 3.Z direction for cutting the point cloud.

## Principle
Do a quick sorting for the point cloud along the designed direction.

Then cut the sorting point cloud into two parts. 

One cloud ranges from Point {1} to Point {(0.5+IoU/2)*PCsize}.

Another cloud ranges from Point {(0.5-IoU/2)*PCsize} to Point {PCsize}.

## About Author
Yue Pan from Wuhan University.

To contact the author, please email to panyue@whu.edu.cn
