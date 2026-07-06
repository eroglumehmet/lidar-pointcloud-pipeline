import numpy as np
import open3d as o3d
import matplotlib as mpl
import matplotlib.pyplot as plt
dataset = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(dataset.path)#indir sonra yükle
voxel_size = 0.02
down = pcd.voxel_down_sample(voxel_size)#indirge
plane_model, inliers = down.segment_plane(0.02, 3, 1000)#zemini bul
objects = down.select_by_index(inliers, invert=True)#zeminin üstündekiler nesne
labels = objects.cluster_dbscan(eps=0.05, min_points=10)#noktaları sınıflandır yakınlığa göre küme numarası numarası -1 olanlar bağımsız yani noise
labels = np.asarray(labels)
max_label = labels.max()
print("küme sayısı:", max_label + 1)#+1 olma sebebi noise
normalized = labels / max(max_label , 1)
colormap = plt.get_cmap('magma')
colors = colormap(normalized)#colormapi çıkar
colors[labels < 0] = 0#noise olanı siyah yap
objects.colors = o3d.utility.Vector3dVector(colors[:, :3])#rgb istediğimiz için 3 yoksa rgba olurdu
o3d.visualization.draw_geometries([objects])
