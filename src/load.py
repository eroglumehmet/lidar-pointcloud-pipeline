import numpy as np
import open3d as o3d
dataset = o3d.data.PLYPointCloud() #veriyi indir
pcd = o3d.io.read_point_cloud(dataset.path) #cpp nesne
print(pcd) #özet basar
print(len(np.asarray(pcd.points)))#numpy cast sonra nokta sayısı yazdır
voxel_size = 0.05
down = pcd.voxel_down_sample(voxel_size) #5 cmlik küplere bölüp her küpte tek nokta bıraktık işleme süresini düşürür
print(len(np.asarray(down.points))) #numpy cast sonra nokta sayısı yazdır
print(pcd.get_axis_aligned_bounding_box())
o3d.visualization.draw_geometries([down])
o3d.visualization.draw_geometries([pcd]) #görselleştir
