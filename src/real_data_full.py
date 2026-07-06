import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import laspy

las = laspy.read("data/autzen.laz")
coordinates = np.column_stack([las.x, las.y, las.z])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(coordinates)

down = pcd.voxel_down_sample(5.0)

plane_model, inliers = down.segment_plane(
        distance_threshold = 15.0,
        ransac_n = 3,
        num_iterations = 1000)
print(plane_model)
print("points after downsample:", len(down.points))

ground = down.select_by_index(inliers)
ground.paint_uniform_color([1, 0, 0])
objects = down.select_by_index(inliers, invert=True)

labels = objects.cluster_dbscan(eps=10, min_points=10, print_progress=True)
labels = np.asarray(labels)
max_label = labels.max()
print("clusters:", max_label + 1)
normalized = labels / max(max_label , 1)
colormap = plt.get_cmap('tab20')
colors = colormap(normalized)
colors[labels < 0] = 0
objects.colors = o3d.utility.Vector3dVector(colors[:, :3])

o3d.visualization.draw_geometries([ground, objects])
