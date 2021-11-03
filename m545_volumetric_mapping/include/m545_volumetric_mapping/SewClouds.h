#include <open3d/Open3D.h>

open3d::geometry::PointCloud SewTwoClouds() {
    open3d::io::ReadPointCloud("", pc1);
    open3d::io::ReadPointCloud("", pc2);
//    maybe downsize here
//    auto pc1_sizedown = *pc1.VoxelDownSample(voxel_sd);
    reg_p2p = open3d::pipelines::registration::RegistrationICP(*pc1, *pc2, threshold,
                                                               trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                                                               open3d::pipelines::registration::ICPConvergenceCriteria(0, 0, 2000));
    evaluation = open3d::pipelines::registration::EvaluateRegistration(*pc1, *pc2, threshold, reg_p2p.transformation_);
    draw_registration_result(*pc1, *pc2, reg_p2p.transformation_);
}