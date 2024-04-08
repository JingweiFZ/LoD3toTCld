import numpy as np

from utils_plane import *
import pyransac3d as pyrsc
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
import FGR
import random
import time


def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=1000):
    """ Detect multiple planes from given point clouds

    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    N = len(points)
    target = points.copy()
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)

    return plane_list

def sort_arrays_by_plane_range(array_of_arrays):
    if len(array_of_arrays) < 2:
        print("Not enough arrays to compare.")
        return None

    # 创建一个列表，其中包含每个数组的索引和对应的平面范围
    ranges = []
    for i in range(len(array_of_arrays)):
        w, points = array_of_arrays[i]

        # 计算凸包
        hull = ConvexHull(points)

        # 获取凸包的顶点索引
        vertices = hull.vertices

        # 获取凸包的顶点坐标
        plane_points = points[vertices]

        # 计算凸包的范围
        plane_range = np.max(plane_points, axis=0) - np.min(plane_points, axis=0)

        # 记录索引和范围
        ranges.append((i, points.shape[0]))#np.prod(plane_range)

    # 按照平面范围从大到小排序
    sorted_ranges = sorted(ranges, key=lambda x: x[1], reverse=True)

    # 打印排序结果
    print("Sorted Arrays by Plane Range:")

    # 返回排序后的数组索引
    sorted_indices = [idx for idx, _ in sorted_ranges]

    plane_list=[]
    for idx in sorted_indices:
        plane_list.append(array_of_arrays[idx])
    return plane_list

def point_to_plane_distance(plane_coefficients, points):
    # 平面系数 (A, B, C, D)
    A, B, C, D = plane_coefficients

    # 平面法向量的模
    plane_normal_magnitude = np.sqrt(A**2 + B**2 + C**2)

    # 计算每个点到平面的距离
    distances = np.abs(A * points[:, 0] + B * points[:, 1] + C * points[:, 2] + D) / plane_normal_magnitude

    return np.mean(distances)

def find_nearest_point_set(points, results):
    # 创建Open3D点云对象
    # points_cloud = o3d.geometry.PointCloud()
    coeff, pointset = points
    # points_cloud.points = o3d.utility.Vector3dVector(pointset)

    min_distance = float('inf')
    nearest_point_set = None

    for idx in range(0, len(results)):
        # 创建Open3D点云对象
        # result_cloud = o3d.geometry.PointCloud()
        # result_cloud.points = results[idx][1]

        # 计算点云之间的距离
        total_distance = point_to_plane_distance(coeff,results[idx][1])

        if(idx==0):
            min_distance=total_distance
            nearest_point_set = results[idx]
        # 更新最小距离和对应的点集
        if total_distance < min_distance:
            min_distance = total_distance
            nearest_point_set = results[idx]

    return nearest_point_set

def visualizedPlanes(pls):
    planes=[]
    colors=[]
    for _, plane in pls:
        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((plane.shape[0], plane.shape[1]))
        color[:, 0] = r
        color[:, 1] = g
        color[:, 2] = b

        planes.append(plane)
        colors.append(color)

    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)
    DrawResult(planes, colors)

if __name__ == "__main__":

    id = f'DEBY_LOD3_4906981'
    save_dir = f'G:/3DGeo/source/additional_data/{id}/'
    #points = ReadPlyPoint('Data/test1.ply')f'{save_dir}/FGR/fgr_lod.pcd'
    cloud2 = o3d.io.read_point_cloud(f'{save_dir}/FGR/fgr_lod.pcd')
    cloud22 = o3d.io.read_point_cloud(f'{save_dir}/FGR/fgr_Tlod.pcd')
    cloud1 = o3d.io.read_point_cloud(f'{save_dir}/Tcd_r.pcd')

    threshold = 0.05
    trans_init = np.identity(4)
    evaluation = o3d.pipelines.registration.evaluate_registration(
        cloud22, cloud1, threshold, trans_init)
    print(evaluation)

    points=cloud1
    points_t = cloud2
    #print(f'num points:{points.shape[0]}')


    # pre-processing
    #points = RemoveNan(points)
    #points = DownSample(points,voxel_size=0.003)
    points = RemoveNoiseStatistical(points, nb_neighbors=15, std_ratio=0.85)
    points_t = RemoveNoiseStatistical(points_t, nb_neighbors=5, std_ratio=0.9)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_t)
    # o3d.visualization.draw_geometries([point_cloud],
    #                                   zoom=0.4559,
    #                                   front=[0.6452, -0.3036, -0.7011],
    #                                   lookat=[1.9892, 2.0208, 1.8945],
    #                                   up=[-0.2779, -0.9482, 0.1556])

    # plane1 = pyrsc.Plane()
    # best_eq, best_inliers = plane1.fit(points, 0.1)

    #DrawPointCloud(points, color=(0.4, 0.4, 0.4))
    t0 = time.time()
    print(f'start:{t0},{points.shape[0]}')

    results_t = DetectMultiPlanes(points_t, min_ratio=0.05, threshold=0.05, iterations=200)
    # visualizedPlanes(results_t)
    results = DetectMultiPlanes(points, min_ratio=0.01, threshold=0.1, iterations=200)
    # visualizedPlanes(results)
    print('Time:', time.time() - t0)
    planes = []
    colors = []

    results1 = sort_arrays_by_plane_range(results)

    pls = []
    #pls.append((results1[0]))

    set1 = find_nearest_point_set(results1[0],results_t)
    # pls.append(set1)

    print('icp registration')
    print("Apply point-to-plane ICP")
    threshold = 0.1
    voxel_size=0.1
    trans_init = np.identity(4)
    point_cloud1 = o3d.geometry.PointCloud()
    point_cloud1.points = o3d.utility.Vector3dVector(results1[0][1])
    point_cloud1 = FGR.voxel_down(point_cloud1,voxel_size)

    point_cloud2 = o3d.geometry.PointCloud()
    point_cloud2.points = o3d.utility.Vector3dVector(set1[1])
    point_cloud2 = FGR.voxel_down(point_cloud2, voxel_size)

    # 计算源点云和目标点云的质心
    source_center = np.asarray(point_cloud2.get_center())
    target_center = np.asarray(point_cloud1.get_center())

    # 计算质心的偏移
    translation_vector = target_center - source_center

    vec = translation_vector/2

    # 将源点云移动到目标点云的质心位置
    point_cloud2 = point_cloud2.translate(translation_vector/2)

    min_bound1 = point_cloud1.get_min_bound()
    min_bound2 = point_cloud2.get_min_bound()

    translation_height = min_bound1 - min_bound2
    print(translation_height)
    translation_height[0]=0
    translation_height[1]=0

    point_cloud2 = point_cloud2.translate(translation_height)
    vec = translation_height + vec

    point_cloud1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
    point_cloud2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))

    # voxel_size=0.1
    # point_cloud1 = FGR.voxel_down(point_cloud1, voxel_size)
    # point_cloud2 = FGR.voxel_down(point_cloud2, voxel_size)
    # source_fpfh = FGR.cal_fpfh(point_cloud1, voxel_size)
    # target_fpfh = FGR.cal_fpfh(point_cloud2, voxel_size)
    # #
    # reg_p2l = FGR.execute_global_registration(point_cloud1, point_cloud2, source_fpfh, target_fpfh,
    #                                  voxel_size)

    reg_p2l = o3d.pipelines.registration.registration_icp(
         point_cloud2, point_cloud1, threshold, trans_init,
         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
         o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=0.1, max_iteration=200))
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)

    source_temp=copy.deepcopy(point_cloud2)
    point_cloud2.transform(reg_p2l.transformation)
    # pls.append()
    pls.append((set1[0], np.asarray(point_cloud2.points)))
    pls.append((set1[0], np.asarray(point_cloud1.points)))

    cl = np.asarray([[255, 178, 24], [0, 85, 255], [255, 178, 24]])
    i=0
    for _, plane in pls:

        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((plane.shape[0], plane.shape[1]))
        color[:, 0] = r
        color[:, 1] = g
        color[:, 2] = b
        #color[i,:]=cl[i,:]
        i = 1+i

        planes.append(plane)
        colors.append(color)
    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)
    DrawResult(planes, colors)


    min_b1 = cloud22.get_min_bound()
    min_b2 = cloud2.get_min_bound()

    theight = min_b2 - min_b1

    # 获取平移矩阵
    translation_matrix = np.eye(4)
    translation_matrix[:3, 3] = vec
    # t_matrix = reg_p2l.transformation
    translation_matrix = np.dot(reg_p2l.transformation, translation_matrix)

    cloud22.transform(translation_matrix)

    o3d.io.write_point_cloud(f'{save_dir}/fine/fine_Tlod.pcd', cloud22)

    output_file = f'{save_dir}/fine/fine_mtx.pcd'
    # 将平移结果保存到文本文件
    np.savetxt(output_file, translation_matrix, fmt='%.6f')

    evaluation = o3d.pipelines.registration.evaluate_registration(
        cloud22, cloud1, threshold, trans_init)
    print(evaluation)



