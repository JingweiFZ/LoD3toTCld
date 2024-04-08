import numpy as np

from FGR import *
import open3d as o3d


if __name__=="__main__":
    print("evaluate the registration results:")

    cloud_tir = o3d.io.read_point_cloud('F:/Downloads/coregistration/pcd_LaserIR_reference_0.pcd')
    #manually files:

    cloud_GT = o3d.io.read_point_cloud('F:/Downloads/coregistration/Tlod3_manually.pcd')
    #FGR files

    cloud_FGR = o3d.io.read_point_cloud('F:/Downloads/coregistration/Tlod3.pcd')
    #fine registratio files

    cloud_fine = o3d.io.read_point_cloud('F:/Downloads/coregistration/fine_Tcloud.pcd')

    source1 = o3d.io.read_point_cloud('F:/Downloads/coregistration/OneDrive/source_thermal_point_cloud.pcd')
    target2 = o3d.io.read_point_cloud('F:/Downloads/coregistration/OneDrive/target_model_point_cloud.pcd')

    threshold = 1
    trans_init = np.identity(4)
    evaluation = o3d.pipelines.registration.evaluate_registration(
        cloud_tir, cloud_GT, threshold, trans_init)
    print(evaluation)

    evaluation = o3d.pipelines.registration.evaluate_registration(
        source1, target2, threshold, trans_init)
    print(evaluation)

    gt_points = np.asarray(cloud_GT.points)
    ft_points = np.asarray(cloud_fine.points)
    min_points = min(len(gt_points), len(ft_points))

    gt_points = gt_points[:min_points, :]
    ft_points = ft_points[:min_points, :]

    print(f'gt last: {gt_points[1,:]}')
    print(f'ft last: {ft_points[1,:]}')

    rmse = np.sqrt(np.mean(np.sum((gt_points - ft_points) ** 2, axis=1)))
    print(f'RMSE is: {rmse}')