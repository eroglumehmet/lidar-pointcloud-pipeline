import numpy as np
import open3d as o3d
import matplotlib as mpl
import matplotlib.pyplot as plt
dataset = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)# download then load
voxel_size = 0.02
down = pcd.voxel_down_sample(voxel_size)# downsample
plane_model, inliers = down.segment_plane(0.02, 3, 1000)# find ground
objects = down.select_by_index(inliers, invert=True)# points above ground = objects
labels = objects.cluster_dbscan(eps=0.05, min_points=10)# cluster by proximity; label -1 = noise
labels = np.asarray(labels)
max_label = labels.max()
print("clusters:", max_label + 1)# +1 since labels are 0-indexed
normalized = labels / max(max_label , 1)
colormap = plt.get_cmap('magma')
colors = colormap(normalized)# get colormap
colors[labels < 0] = 0# noise -> black
objects.colors = o3d.utility.Vector3dVector(colors[:, :3])# take 3 cols (rgb), not rgba
o3d.visualization.draw_geometries([objects])
