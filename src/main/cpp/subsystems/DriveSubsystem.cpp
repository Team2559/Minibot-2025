#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

#include <rev/config/SparkMaxConfig.h>

using namespace DriveConstants;
using namespace rev::spark;

DriveSubsystem::DriveSubsystem() :
  frontLeftMotor{kFrontLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  frontRightMotor{kFrontRightDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  rearLeftMotor{kRearLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  rearRightMotor{kRearRightDriveID, rev::spark::SparkMax::MotorType::kBrushless}
{
  {
    SparkMaxConfig driveConfig;
    driveConfig
      .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
      .SmartCurrentLimit(40.0)
      .Inverted(false);

    driveConfig.encoder
      .PositionConversionFactor(kDriveDistancePerRotation.value())
      .VelocityConversionFactor(kDriveDistancePerRotation.value() * (1.0_s / 1.0_min));

    driveConfig.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pid(DrivePID::kP, DrivePID::kI, DrivePID::kD);
    driveVff = DrivePID::kV;

    frontLeftMotor.Configure(driveConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    rearLeftMotor.Configure(driveConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);

    // Right side motors need to spin the other way to go forwards
    driveConfig.Inverted(true);

    frontRightMotor.Configure(driveConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    rearRightMotor.Configure(driveConfig, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }
}

void DriveSubsystem::drive(frc::ChassisSpeeds speed, bool fieldRelative) {
  if (fieldRelative) {
    frc::Rotation2d angle = 0.0_rad;
    speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speed, angle);
  }
  frc::MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.ToWheelSpeeds(speed);

  frontRightMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.frontRight.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.frontRight).value()
  );
  
  frontLeftMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.frontLeft.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.frontLeft).value()
  );
  
  rearRightMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.rearRight.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.rearRight).value()
  );
    
  rearLeftMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.rearLeft.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.rearLeft).value()
  );
}

void DriveSubsystem::stop() {
  frontLeftMotor.StopMotor();
  frontRightMotor.StopMotor();
  rearLeftMotor.StopMotor();
  rearRightMotor.StopMotor();
}

