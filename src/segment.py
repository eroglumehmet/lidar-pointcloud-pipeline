import numpy as np
import open3d as o3d
dataset = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)
down = pcd.voxel_down_sample(0.02)
plane_model, inliers = down.segment_plane(
        distance_threshold = 0.02,
        ransac_n = 3,
        num_iterations = 1000)
ground = down.select_by_index(inliers)
objects = down.select_by_index(inliers, invert=True)
ground.paint_uniform_color([1, 0, 0])
objects.paint_uniform_color([0, 0, 1])
print(plane_model)
o3d.visualization.draw_geometries([ground , objects])
