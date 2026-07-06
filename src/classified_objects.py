import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import laspy
from collections import Counter

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
print("downsample sonrası nokta sayısı:", len(down.points))

ground = down.select_by_index(inliers)
ground.paint_uniform_color([1, 0, 0])
objects = down.select_by_index(inliers, invert=True)

labels = objects.cluster_dbscan(eps=10, min_points=10, print_progress=True)
labels = np.asarray(labels)
max_label = labels.max()
print("küme sayısı:", max_label + 1)

colors = np.zeros((len(objects.points), 3))
class_color = {"pole":[1,1,0], "building":[0,0,1], "vegetation":[0,1,0]}
counts = Counter()

for k in range(max_label + 1):
    mask = (labels == k)
    cl_points = np.asarray(objects.points)[mask]
    if len(cl_points) < 10:
        continue
    height = cl_points[:, 2].max() - cl_points[:, 2].min()
    x_spread = cl_points[:, 0].max() - cl_points[:, 0].min()
    y_spread = cl_points[:, 1].max() - cl_points[:, 1].min()
    footprint = max(x_spread, y_spread)
    center = cl_points.mean(axis=0)
    centered_data = cl_points - center
    covariance = np.cov(centered_data.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    linearity = (eigenvalues[2] - eigenvalues[1]) / (eigenvalues[2] + 1e-9)
    planarity = (eigenvalues[1] - eigenvalues[0]) / (eigenvalues[2] + 1e-9)
    sphericity = eigenvalues[0] / (eigenvalues[2] + 1e-9)
    v0 = eigenvectors[:, 2]
    verticality = abs(v0[2])
    if verticality > 0.85 and linearity > 0.5 and height > 30:
        label = "pole"
    elif footprint > 100 and verticality < 0.4:
        label = "building"
    elif sphericity > 0.30:
        label = "vegetation"
    else:
        label = "vegetation"
    colors[mask] = class_color[label]
    counts[label] += 1
    if len(cl_points) > 50:
        print(f"k={k:3d}  n={len(cl_points):4d}  h={height:6.1f}  fp={footprint:6.1f}  "
          f"lin={linearity:.2f}  plan={planarity:.2f}  sph={sphericity:.2f}  vert={verticality:.2f}")
          
objects.colors = o3d.utility.Vector3dVector(colors)
print(counts)
scene = ground + objects
o3d.io.write_point_cloud("output/autzen_classified.ply", scene)
o3d.visualization.draw_geometries([ground, objects])
