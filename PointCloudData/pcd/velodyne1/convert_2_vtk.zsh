#!/bin/zsh

for files in $(find . -name "*.pcd" -type f)
do
	pcl_pcd2vtk "$files" "${files%.*}.vtk"
done
