#pragma once

#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/GenericEntry.h>
#include <rev/SparkMax.h>
//local
#include "Constants.h"
#include "PIDTuner.h"

class DriveSubsystem : public frc2::SubsystemBase {
  public:
    DriveSubsystem();

    void Periodic() override;

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

    PIDTuner wheelSpeedTuner;

    static void graphClosedLoop(rev::spark::SparkMax &motor);

    // TODO: Add wheel velocity PID tuner + setpoint/output graph
    // TODO: Add pose estimation
    // TODO: Add IMU for heading feedback
    // TODO: Add chassis position PID loops
    // TODO: Add Choreo path following

    double driveVff;

    nt::GenericEntry *nt_flVelocity;
    nt::GenericEntry *nt_flSetpoint;
    nt::GenericEntry *nt_flOutput;    
    
    nt::GenericEntry *nt_frVelocity;
    nt::GenericEntry *nt_frSetpoint;
    nt::GenericEntry *nt_frOutput;    
    
    nt::GenericEntry *nt_rlVelocity;
    nt::GenericEntry *nt_rlSetpoint;
    nt::GenericEntry *nt_rlOutput;

    nt::GenericEntry *nt_rrVelocity;
    nt::GenericEntry *nt_rrSetpoint;
    nt::GenericEntry *nt_rrOutput;
};
