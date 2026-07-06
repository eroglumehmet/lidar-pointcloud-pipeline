import numpy as np
import open3d as o3d
dataset = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)
voxel_size = 0.02
down = pcd.voxel_down_sample(voxel_size)
clean, ind = down.remove_statistical_outlier(
        nb_neighbors = 20,
        std_ratio = 2.0)
outliers = down.select_by_index(ind, invert=True) # removed points
outliers.paint_uniform_color([1, 0, 0])# color removed points
clean.paint_uniform_color([0.6, 0.6, 0.6])# color clean points
o3d.visualization.draw_geometries([clean, outliers])
print("removed points:", len(down.points) - len(clean.points))
