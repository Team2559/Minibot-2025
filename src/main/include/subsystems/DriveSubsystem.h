#pragma once

#include <frc/geometry/Pose3d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/estimator/MecanumDrivePoseEstimator3d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <rev/SparkMax.h>
#include <studica/AHRS.h>
//local
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

    frc::MecanumDriveKinematics driveKinematics {
        DriveConstants::kFrontLeftWheel,
        DriveConstants::kFrontRightWheel,
        DriveConstants::kBackLeftWheel,
        DriveConstants::kBackRightWheel,
    };
  private:
    rev::spark::SparkMax frontLeftMotor;
    rev::spark::SparkMax frontRightMotor;
    rev::spark::SparkMax rearLeftMotor;
    rev::spark::SparkMax rearRightMotor;

    studica::AHRS ahrs;

    frc::MecanumDrivePoseEstimator3d poseEstimator;

    PIDTuner wheelSpeedTuner;

    static void graphClosedLoop(rev::spark::SparkMax &motor);

    // TODO: Add chassis position PID loops
    // TODO: Add Choreo path following

    double driveVff;

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
};
