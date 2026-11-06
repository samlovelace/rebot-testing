// Standalone validation: compares the analytic Jacobian against a
// finite-difference numerical Jacobian computed from forward kinematics.
// If these agree, the FK and Jacobian math is internally consistent.

#include "ArmKinematics.hpp"

#include <cstdio>
#include <random>

using namespace motorbridge;

namespace {

ArmJacobian numericJacobian(const ArmKinematics& kin, const JointVec& q, double eps = 1e-6) {
    ArmJacobian J;
    const Eigen::Isometry3d t0 = kin.forwardKinematics(q);
    for (int i = 0; i < kNumArmJoints; ++i) {
        JointVec qp = q;
        qp(i) += eps;
        const Eigen::Isometry3d t1 = kin.forwardKinematics(qp);

        J.block<3, 1>(0, i) = (t1.translation() - t0.translation()) / eps;

        // angular velocity from the small relative rotation t0^-1 * t1
        Eigen::Matrix3d dR = t0.linear().transpose() * t1.linear();
        Eigen::AngleAxisd aa(dR);
        Eigen::Vector3d w_local = aa.axis() * aa.angle() / eps;
        J.block<3, 1>(3, i) = t0.linear() * w_local;   // rotate back to base frame
    }
    return J;
}

} // namespace

int main() {
    ArmKinematics kin;

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    double maxErr = 0.0;
    for (int trial = 0; trial < 20; ++trial) {
        JointVec q;
        for (int i = 0; i < kNumArmJoints; ++i) q(i) = dist(rng);

        ArmJacobian Ja = kin.jacobian(q);
        ArmJacobian Jn = numericJacobian(kin, q);
        double err = (Ja - Jn).cwiseAbs().maxCoeff();
        maxErr = std::max(maxErr, err);
        std::printf("trial %2d: max |analytic - numeric| = %.3e\n", trial, err);
    }

    std::printf("\nworst case over 20 random configs: %.3e\n", maxErr);

    JointVec q0 = JointVec::Zero();
    Eigen::Isometry3d t0 = kin.forwardKinematics(q0);
    std::printf("\nFK at q=0: tool position = [%.4f, %.4f, %.4f] m\n",
                t0.translation().x(), t0.translation().y(), t0.translation().z());

    // Resolved-rate sanity check, isolated from the damping/accuracy tradeoff:
    // with near-zero lambda, J * qdot should reproduce the commanded twist
    // almost exactly if the DLS solve itself is implemented correctly.
    // (lambda=0.05 at runtime deliberately sacrifices some of this accuracy
    // for safety near singularities -- that tradeoff is checked separately below.)
    double maxTwistErr = 0.0;
    for (int trial = 0; trial < 10; ++trial) {
        JointVec q;
        for (int i = 0; i < kNumArmJoints; ++i) q(i) = dist(rng) * 0.5;
        Twist twist;
        for (int i = 0; i < 6; ++i) twist(i) = dist(rng) * 0.1;

        JointVec qdot = kin.resolvedRateJointVelocity(q, twist, /*lambda=*/1e-6);
        Twist achieved = kin.jacobian(q) * qdot;
        double err = (achieved - twist).cwiseAbs().maxCoeff();
        maxTwistErr = std::max(maxTwistErr, err);
    }
    std::printf("resolved-rate IK (lambda~0): worst twist tracking error over 10 random commands = %.3e\n",
                maxTwistErr);

    // Runtime damping (lambda=0.05) must stay bounded (no blowup) even when
    // deliberately evaluated at q=0, which is this arm's most singular-prone
    // reference pose (fully extended along the chain).
    Twist unitTwist;
    unitTwist << 1, 0, 0, 0, 0, 0;
    JointVec qdotDamped = kin.resolvedRateJointVelocity(q0, unitTwist, /*lambda=*/0.05);
    double qdotNorm = qdotDamped.norm();
    std::printf("damped IK at q=0 for unit twist: |qdot| = %.3f rad/s (should be bounded, not huge)\n",
                qdotNorm);

    if (maxErr < 1e-4 && maxTwistErr < 1e-2 && qdotNorm < 100.0) {
        std::printf("PASS: analytic Jacobian matches finite-difference FK\n");
        return 0;
    }
    std::printf("FAIL: Jacobian does not match finite-difference FK\n");
    return 1;
}
