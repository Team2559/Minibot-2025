// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/PoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include "Constants.h"

/**
 * Processes camera data to provide robot pose estimates from visible AprilTags
 */
class VisionSubsystem : public frc2::SubsystemBase {
  public:
    VisionSubsystem(std::function<frc::Pose3d()> accessor, std::function<void(frc::Pose3d, units::millisecond_t)> updater);

    /**
     * Returns an initial pose estimate based on the first valid camera result, if any tags are visible
     */
    std::optional<frc::Pose3d> SeedPose();

    void Periodic() override;

  private:
    /// Camera for pose estimation
    photon::PhotonCamera m_camera;
    /// Pose estimator to turn 2d tag coordinates into 3d robot poses
    photon::PhotonPoseEstimator m_poseEstimator;

    /// Accessor for the current robot pose
    std::function<frc::Pose3d()> m_accessor;
    /// Updater for new robot poses
    std::function<void(frc::Pose3d, units::millisecond_t)> m_updater;

    /// Provides new pose estimates based on all unused camera results, if any
    void ProcessCameraResults(std::function<void(frc::Pose3d, units::millisecond_t)> updater);
};
