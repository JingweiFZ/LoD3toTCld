import time

import open3d as o3d
import numpy as np
import copy


def read_pcd(filename):
    print(f'read {filename}')
    pcd = o3d.io.read_point_cloud(filename)
    return pcd

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result

def voxel_down(pcd,voxel_size):
    print(f'voxelization down, voxel size:  {voxel_size}')
    pcd_down = pcd.voxel_down_sample(voxel_size)
    return pcd_down

def cal_fpfh(pcd,voxel_size):
    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def FGR(cloud1, cloud2, voxel_size):
    print(f'Fast globale registration')
    start = time.time()
    source_fpfh = cal_fpfh(cloud1, voxel_size)
    target_fpfh = cal_fpfh(cloud2, voxel_size)
    result_ransac = execute_global_registration(cloud1, cloud2,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
    return result_ransac

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

if __name__ == "__main__":
    print(f'point cloud registration')

    cloud2 = read_pcd('F:/Downloads/coregistration/FGR_lod3_new.pcd')
    cloud1 = read_pcd('F:/Downloads/coregistration/pcd_LaserIR_reference_0.pcd')

    cloud2, ind = cloud2.remove_statistical_outlier(nb_neighbors = 20, std_ratio=2.0)

    voxel_size = 0.2
    cld1_down = voxel_down(cloud1, voxel_size)
    cld2_down = voxel_down(cloud2, voxel_size)
    #
    # print(f'fast global registration')
    # result_ransac = FGR(cld1_down, cld2_down, voxel_size)
    # draw_registration_result(cld1_down, cld2_down, result_ransac.transformation)

    #print("Global registration took %.3f sec.\n" % (time.time() - start))

    cld1_fpfh = cal_fpfh(cld1_down, voxel_size)
    cld2_fpfh = cal_fpfh(cld2_down, voxel_size)
    start = time.time()
    result_ransac = execute_fast_global_registration(cld1_down, cld2_down,
                                                   cld1_fpfh, cld2_fpfh,
                                                   voxel_size)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    print(result_ransac)
    draw_registration_result(cld1_down, cld2_down,
                             result_ransac.transformation)

    print(f'fine registration')
    source_fpfh = cal_fpfh(cloud1, voxel_size)
    target_fpfh = cal_fpfh(cloud2, voxel_size)
    result_icp = refine_registration(cloud1, cloud2, source_fpfh, target_fpfh,
                                     voxel_size)

    print(result_icp)
    draw_registration_result(cloud1, cloud2, result_icp.transformation)

    print(f'finish!')
