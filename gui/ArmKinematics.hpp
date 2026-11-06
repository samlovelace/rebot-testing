#pragma once
// Forward kinematics + geometric Jacobian for the Seeed reBot Arm B601-DM
// (6R serial chain + fixed tool offset). Geometry below is taken directly
// from the arm's own ReBot_Arm_DM.urdf (joint1..joint6 <origin>/<axis> and
// end_joint), base_link -> link1..link6 -> end_link. The two gripper finger
// joints are NOT part of this chain -- the gripper is driven independently.
//
// Source: https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_DM
//   reBotArm_ros2_DM/src/rebotarm_bringup/description/urdf/ReBot_Arm_DM.urdf

#include <Eigen/Dense>
#include <array>

namespace motorbridge {

constexpr int kNumArmJoints = 6;
using JointVec = Eigen::Matrix<double, kNumArmJoints, 1>;
using Twist = Eigen::Matrix<double, 6, 1>;   // [vx,vy,vz, wx,wy,wz]
using ArmJacobian = Eigen::Matrix<double, 6, kNumArmJoints>;

struct JointGeom {
    Eigen::Vector3d origin_xyz;
    Eigen::Vector3d rpy;    // roll, pitch, yaw of <origin>
    Eigen::Vector3d axis;   // joint rotation axis, in the post-origin frame
};

class ArmKinematics {
public:
    ArmKinematics();

    // Pose of the tool frame (end_link) in base_link for joint angles q.
    Eigen::Isometry3d forwardKinematics(const JointVec& q) const;

    // Geometric Jacobian in base_link frame: maps joint velocity to the
    // tool frame origin's [linear; angular] velocity, expressed in base_link.
    ArmJacobian jacobian(const JointVec& q) const;

    // Damped least-squares resolved-rate IK: given a desired twist in
    // base_link frame, returns joint velocities. Larger lambda trades
    // tracking accuracy for stability near singularities (bounds the
    // velocity spike instead of letting a plain pseudo-inverse blow up).
    JointVec resolvedRateJointVelocity(const JointVec& q, const Twist& twist_base,
                                        double lambda = 0.05) const;

private:
    std::array<JointGeom, kNumArmJoints> joints_;
    Eigen::Isometry3d toolOffset_;   // end_joint: link6 -> end_link (fixed)
};

} // namespace motorbridge
