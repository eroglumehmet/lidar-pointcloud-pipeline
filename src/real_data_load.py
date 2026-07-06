import numpy as np
import open3d as o3d
import laspy
las = laspy.read("data/autzen.laz")
coordinates = np.column_stack([las.x, las.y, las.z])# pull points from file
pcd = o3d.geometry.PointCloud()# create object
pcd.points = o3d.utility.Vector3dVector(coordinates)# fill with real data
print(pcd)
print(pcd.get_axis_aligned_bounding_box())# print bounds
o3d.visualization.draw_geometries([pcd])# draw
