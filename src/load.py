import numpy as np
import open3d as o3d
dataset = o3d.data.PLYPointCloud() # download data
pcd = o3d.io.read_point_cloud(dataset.path) # point cloud object
print(pcd) # prints summary
print(len(np.asarray(pcd.points)))# point count after numpy cast
voxel_size = 0.05
down = pcd.voxel_down_sample(voxel_size) # 5 cm voxels, one point per cube -> fewer points, faster
print(len(np.asarray(down.points))) # point count after numpy cast
print(pcd.get_axis_aligned_bounding_box())
o3d.visualization.draw_geometries([down])
o3d.visualization.draw_geometries([pcd]) # visualize
