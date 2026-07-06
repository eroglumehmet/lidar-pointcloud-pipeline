import numpy as np
import open3d as o3d
import laspy

las = laspy.read("data/autzen.laz")
coordinates = np.column_stack([las.x, las.y, las.z])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(coordinates)

down = pcd.voxel_down_sample(5.0)
points = np.asarray(down.points)
np.savetxt("cpp/data/cloud.csv", points, delimiter=",")
print("points written:", len(points))
