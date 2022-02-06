# pair_feature_colored_icp

### Building for source:
```sh
$ mkdir build
$ cd build
$ cmake .. && make -j8
```

### Running the sample:
```sh
$ ./pair_feature_colored_icp ../data/sofa/cloud_bin_0.pcd ../data/sofa/cloud_bin_1.pcd -v 1
visualize correspondences: true
radius scale | cloud source: 0.0851716 , cloud target: 0.0607552
Harris detector with normal: 
number of source keypoints found: 186
number of target keypoints found: 117
FPFHEstimationOMP
Matching with Kd-tree...
refining matching...
------------------------------------------------------
pre-IterativeClosestPointWithNormals:
0 0 0 0
0 0 0 0
0 0 0 0
0 0 0 0
pos-IterativeClosestPointWithNormals:
 0.732575 0.0556007 -0.678412   1.69052
-0.298211  0.922136 -0.246444   1.26394
 0.611886  0.382848  0.692115  -1.61844
        0         0         0         1
------------------------------------------------------
pre-estimateRigidTransformation SVD: 
 0.732575 0.0556007 -0.678412   1.69052
-0.298211  0.922136 -0.246444   1.26394
 0.611886  0.382848  0.692115  -1.61844
        0         0         0         1
pos-estimateRigidTransformation SVD: 
  0.850614 0.00452149  -0.533787   0.611311
 -0.140561   0.970659  -0.215768   0.776143
  0.514968   0.257474   0.822805    -1.4751
         0          0          0          1
------------------------------------------------------
estimated scale: 1.00424
estimated rotation: 
  0.850614 0.00452149  -0.533787
 -0.140561   0.970659  -0.215768
  0.514968   0.257474   0.822805
final transformation: 
  0.850614 0.00452149  -0.533787   0.611311
 -0.140561   0.970659  -0.215768   0.776143
  0.514968   0.257474   0.822805    -1.4751
         0          0          0          1
Saved cloud: pair_icp.pcd
```
### Correspondences:

| Capture 1     | Capture 2     |
| ------------- | ------------- |
| ![alt-text-1](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/correspondences1.png) | ![alt-text-2](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/correspondences2.png) 

```sh
$ ./pair_feature_colored_icp pair_icp.pcd ../data/sofa/cloud_bin_2.pcd -v 1
```

### Correspondences:

| Capture 1     | Capture 2     |
| ------------- | ------------- |
| ![alt-text-1](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/correspondences3.png) | ![alt-text-2](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/correspondences4.png) 

### ICP registration:

| Points 1     | Points 2     |
| ------------- | ------------- |
| ![alt-text-1](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/points1.png) | ![alt-text-2](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/points2.png) 

| Final 1     | Final 2     |
| ------------- | ------------- |
| ![alt-text-1](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/final1.png) | ![alt-text-2](https://github.com/vicent3rod/pair_feature_colored_icp/blob/main/captures/final2.png)
