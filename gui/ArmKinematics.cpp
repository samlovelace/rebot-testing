#include "ArmKinematics.hpp"

namespace motorbridge {

namespace {

Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d& rpy) {
    // URDF <origin rpy="r p y"> convention: R = Rz(y) * Ry(p) * Rx(r).
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Isometry3d originTransform(const JointGeom& j) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = rpyToRotation(j.rpy);
    t.translation() = j.origin_xyz;
    return t;
}

} // namespace

ArmKinematics::ArmKinematics() {
    joints_[0] = {{-8.416e-05, 0.0, 0.08465}, {0, 0, 0}, {0, 0, 1}};
    joints_[1] = {{0.020084, 0.031625, 0.05555}, {-1.5708, 0, 0}, {0, 0, -1}};
    joints_[2] = {{-0.264, 0.0, 0.0}, {0, 0, 0}, {0, 0, 1}};
    joints_[3] = {{0.2426, -0.054, -0.001625}, {0, 0, 0}, {0, 0, 1}};
    joints_[4] = {{0.078308, -0.0375, -0.03}, {-1.5708, 0, 0}, {0, 0, 1}};
    joints_[5] = {{0.028008, 0.0, 0.04}, {0, 1.5708, 0}, {0, 0, 1}};

    JointGeom endJoint{{0.0, 0.0, 0.15539}, {0, -1.5708, 3.1415}, {0, 0, 0}};
    toolOffset_ = originTransform(endJoint);
}

Eigen::Isometry3d ArmKinematics::forwardKinematics(const JointVec& q) const {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kNumArmJoints; ++i) {
        t = t * originTransform(joints_[i]);
        t.linear() = t.linear() * Eigen::AngleAxisd(q(i), joints_[i].axis.normalized()).toRotationMatrix();
    }
    return t * toolOffset_;
}

ArmJacobian ArmKinematics::jacobian(const JointVec& q) const {
    // Standard revolute-chain geometric Jacobian:
    //   Jv_i = z_i x (p_ee - p_i),   Jw_i = z_i
    // where p_i/z_i are joint i's origin position and rotation axis, both
    // expressed in base_link at the given configuration.
    std::array<Eigen::Vector3d, kNumArmJoints> p_i;
    std::array<Eigen::Vector3d, kNumArmJoints> z_i;

    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    for (int i = 0; i < kNumArmJoints; ++i) {
        t = t * originTransform(joints_[i]);
        p_i[i] = t.translation();
        z_i[i] = t.linear() * joints_[i].axis.normalized();
        t.linear() = t.linear() * Eigen::AngleAxisd(q(i), joints_[i].axis.normalized()).toRotationMatrix();
    }
    const Eigen::Vector3d p_ee = (t * toolOffset_).translation();

    ArmJacobian J;
    for (int i = 0; i < kNumArmJoints; ++i) {
        J.block<3, 1>(0, i) = z_i[i].cross(p_ee - p_i[i]);
        J.block<3, 1>(3, i) = z_i[i];
    }
    return J;
}

JointVec ArmKinematics::resolvedRateJointVelocity(const JointVec& q, const Twist& twist_base,
                                                    double lambda) const {
    const ArmJacobian J = jacobian(q);
    const Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    const Eigen::Matrix<double, 6, 6> damped =
        JJt + (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
    return J.transpose() * damped.inverse() * twist_base;
}

} // namespace motorbridge
