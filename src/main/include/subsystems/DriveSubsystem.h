// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <choreo/Choreo.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/MecanumDrivePoseEstimator3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <rev/SparkMax.h>
#include <studica/AHRS.h>

// local
#include "Constants.h"
#include "PIDTuner.h"

class DriveSubsystem : public frc2::SubsystemBase {
  public:
    DriveSubsystem();

    void Periodic() override;

    void TestInit();

    /**
     * Drive at approximately the requested speed
     */
    void drive(frc::ChassisSpeeds speed, bool fieldRelative);

    /**
     * Move towards the choreo trajectory sample position (with PID feedback)
     */
    void followTrajectory(const choreo::SwerveSample &sample);

    /**
     * Stop all drivetrain movement
     */
    void stop();

    /**
     *  Reset forward for the driver to be the way the robot is currently facing
     */
    void resetFieldOrientation(bool inverted);

    /**
     * Reset the robot's pose to the provided pose
     */
    void resetPose(frc::Pose3d pose);

    /**
     * Gets the robot's current pose (position + orientation)
     */
    frc::Pose3d getPose();

    /**
     * Incorporate a vision pose measurement into the robots cumulative pose estimation
     */
    void updateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp);

    /**
     * Get the current wheel positions for odometry
     */
    const frc::MecanumDriveWheelPositions getWheelPositions();

    frc::MecanumDriveKinematics driveKinematics{
      DriveConstants::kFrontLeftWheel,
      DriveConstants::kFrontRightWheel,
      DriveConstants::kBackLeftWheel,
      DriveConstants::kBackRightWheel,
    };

  private:
    // The four mecanum wheels
    rev::spark::SparkMax frontLeftMotor;
    rev::spark::SparkMax frontRightMotor;
    rev::spark::SparkMax rearLeftMotor;
    rev::spark::SparkMax rearRightMotor;

    // The navX gyro sensor.
    studica::AHRS ahrs;

    // Pose estimator combines odometry with vision readings to yield an accurate robot pose
    frc::MecanumDrivePoseEstimator3d poseEstimator;

    // Tuners to adjust PID values live from the dashboard; greatly increases the ease of tuning
    PIDTuner wheelSpeedTuner;

    // PID controllers for autonomous path following
    frc::PIDController xController;
    frc::PIDController yController;
    frc::PIDController rController;

    static void graphClosedLoop(rev::spark::SparkMax &motor);

    // TODO: Add chassis position PID loops
    // TODO: Add Choreo path following

    double driveVff;

    // Individual wheel velocity graphing
    nt::DoublePublisher nt_flVelocity;
    nt::DoublePublisher nt_flSetpoint;
    nt::DoublePublisher nt_flOutput;

    nt::DoublePublisher nt_frVelocity;
    nt::DoublePublisher nt_frSetpoint;
    nt::DoublePublisher nt_frOutput;

    nt::DoublePublisher nt_rlVelocity;
    nt::DoublePublisher nt_rlSetpoint;
    nt::DoublePublisher nt_rlOutput;

    nt::DoublePublisher nt_rrVelocity;
    nt::DoublePublisher nt_rrSetpoint;
    nt::DoublePublisher nt_rrOutput;

    // Chassis position graphing
    nt::DoublePublisher nt_xPosition;
    nt::DoublePublisher nt_xSetpoint;
    nt::DoublePublisher nt_xOutput;

    nt::DoublePublisher nt_yPosition;
    nt::DoublePublisher nt_ySetpoint;
    nt::DoublePublisher nt_yOutput;

    nt::DoublePublisher nt_rPosition;
    nt::DoublePublisher nt_rSetpoint;
    nt::DoublePublisher nt_rOutput;
};
