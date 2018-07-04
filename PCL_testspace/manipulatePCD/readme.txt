cd build
cmake ..
make

./centerPCD input.pcd output.pcd

./mergePCD input1.pcd input2.pcd output.pcd i2xShift* i2yShift* i2zShift*
*optional

./cutPCD input.pcd output.pcd xMin xMax yMin yMax zMin zMax
