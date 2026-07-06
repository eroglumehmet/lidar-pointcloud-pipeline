import numpy as np
import open3d as o3d
dataset = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)
voxel_size = 0.02
down = pcd.voxel_down_sample(voxel_size)
clean, ind = down.remove_statistical_outlier(
        nb_neighbors = 20,
        std_ratio = 2.0)
outliers = down.select_by_index(ind, invert=True) #atılan noktalar
outliers.paint_uniform_color([1, 0, 0])#atılan noktaları boya
clean.paint_uniform_color([0.6, 0.6, 0.6])#temiz noktaları boya
o3d.visualization.draw_geometries([clean, outliers])
print("atılan nokta sayısı:", len(down.points) - len(clean.points))
