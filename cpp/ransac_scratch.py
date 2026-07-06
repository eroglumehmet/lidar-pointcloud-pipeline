import numpy as np
import open3d as o3d
import random
import laspy
import time
def ransac_plane(points, threshold, num_iter):
    best_inlier_num = 0
    best_plane = None
    best_mask = None
    for _ in range(num_iter):
        i, j, k = random.sample(range(len(points)), 3)
        p0, p1, p2 = points[i], points[j], points[k]
        n = np.cross(p1-p0, p2-p0)
        if np.linalg.norm(n) < 1e-6:
            continue
        d = -np.dot(n , p0)
        distances = np.abs(points @ n + d) / np.linalg.norm(n)
        inlier_mask = distances < threshold
        count = inlier_mask.sum()
        if count > best_inlier_num:
            best_inlier_num = count
            best_plane = (n, d)
            best_mask = inlier_mask
    return best_plane, best_mask
if __name__ == "__main__":
    las = laspy.read("data/autzen.laz")
    coordinates = np.column_stack([las.x, las.y, las.z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(coordinates)

    down = pcd.voxel_down_sample(5.0)
    points = np.asarray(down.points)
    
    t0 = time.perf_counter()
    best_plane, best_mask = ransac_plane(points, threshold=15.0, num_iter=1000)
    t1 = time.perf_counter()
    n, d = best_plane
    print("normal (a,b,c):", n / np.linalg.norm(n))
    print("inliers:", best_mask.sum())
    print("time (ms):", (t1 - t0) * 1000)
    t0 = time.perf_counter()
    plane_model, inliers = down.segment_plane(distance_threshold=15.0, ransac_n=3, num_iterations=1000)
    t1 = time.perf_counter()
    print("open3d normal:", plane_model[:3])
    print("open3d inlier:", len(inliers))
    print("open3d time (ms):", (t1 - t0) * 1000)
    
        
