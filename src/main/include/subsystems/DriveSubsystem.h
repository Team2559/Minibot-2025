#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
//local
#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
  public:
    DriveSubsystem();

    void drive(frc::ChassisSpeeds speed, bool fieldRelative);
    void stop();

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

    double driveVff;
};
