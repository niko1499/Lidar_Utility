cd build
cmake ..
make
./global_hypothesis_verification ~/Lidar_Utility/PointCloudData/objects/car5.pcd 2321obj.pcd --algorithm Hough --model_ss .25 scene_ss .75 --rf_rad .25 --descr_rad .2 --cg_size 2 --cg_thresh 8


./global_hypothesis_verification ~/Lidar_Utility/PCL_testspace/mergePCD/merged2.pcd 2321obj.pcd --algorithm Hough --model_ss .25 scene_ss .75 --rf_rad .25 --descr_rad .2 --cg_size 2 --cg_thresh 4

./global_hypothesis_verification ~/Lidar_Utility/PCL_testspace/mergePCD/build/merged3.pcd 2321obj.pcd --model_ss .2 scene_ss .4 --rf_rad .3 --descr_rad .6 --cg_size 1.25 --cg_thresh 2.75

./global_hypothesis_verification ~/Lidar_Utility/PCL_testspace/mergePCD/build/car3.pcd 2321obj.pcd --model_ss .2 scene_ss .4 --rf_rad .3 --descr_rad .6 --cg_size 1.25 --cg_thresh 2.75

