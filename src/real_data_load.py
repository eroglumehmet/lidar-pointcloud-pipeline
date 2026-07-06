import numpy as np
import open3d as o3d
import laspy
las = laspy.read("data/autzen.laz")
coordinates = np.column_stack([las.x, las.y, las.z])#dosyadan noktaları çek
pcd = o3d.geometry.PointCloud()#nesne oluştur
pcd.points = o3d.utility.Vector3dVector(coordinates)#gerçek veriyle doldur
print(pcd)
print(pcd.get_axis_aligned_bounding_box())#sınırları yazdır
o3d.visualization.draw_geometries([pcd])#çiz
