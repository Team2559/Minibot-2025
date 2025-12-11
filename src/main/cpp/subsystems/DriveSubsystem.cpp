#include "subsystems/DriveSubsystem.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/config/SparkMaxConfig.h>

using namespace DriveConstants;
using namespace rev::spark;

DriveSubsystem::DriveSubsystem() :
  frontLeftMotor{kFrontLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  frontRightMotor{kFrontRightDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  rearLeftMotor{kRearLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  rearRightMotor{kRearRightDriveID, rev::spark::SparkMax::MotorType::kBrushless},
  ahrs{studica::AHRS::NavXComType::kMXP_SPI},
  poseEstimator{driveKinematics, ahrs.GetRotation3d(), getWheelPositions(), frc::Pose3d()},
  wheelSpeedTuner{[this](PIDUpdate update) {
    SparkMaxConfig config;
    switch (update.term) {
      case PIDUpdate::PIDTerm::kP:
        config.closedLoop.P(update.value, ClosedLoopSlot(update.slot));
        break;
      case PIDUpdate::PIDTerm::kI:
        config.closedLoop.I(update.value, ClosedLoopSlot(update.slot));
        break;
      case PIDUpdate::PIDTerm::kD:
        config.closedLoop.D(update.value, ClosedLoopSlot(update.slot));
        break;
      case PIDUpdate::PIDTerm::kFF:
        driveVff = update.value;
        return;
    }

    frontLeftMotor.Configure(config, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    frontRightMotor.Configure(config, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    rearLeftMotor.Configure(config, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
    rearRightMotor.Configure(config, SparkMax::ResetMode::kNoResetSafeParameters, SparkMax::PersistMode::kNoPersistParameters);
  }}
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
  {
    auto ntInstance = nt::NetworkTableInstance::GetDefault();
    auto table = ntInstance.GetTable("Drive");

    nt_flVelocity = table->GetDoubleTopic("flVelocity").Publish();
    nt_flOutput = table->GetDoubleTopic("flOutput").Publish();
    nt_flSetpoint = table->GetDoubleTopic("flSetpoint").Publish();

    nt_frVelocity = table->GetDoubleTopic("frVelocity").Publish();
    nt_frOutput = table->GetDoubleTopic("frOutput").Publish();
    nt_frSetpoint = table->GetDoubleTopic("frSetpoint").Publish();

    nt_rlOutput = table->GetDoubleTopic("rlOutput").Publish();
    nt_rlVelocity = table->GetDoubleTopic("rlVelocity").Publish();
    nt_rlSetpoint = table->GetDoubleTopic("rlSetpoint").Publish();

    nt_rrOutput = table->GetDoubleTopic("rrOutput").Publish();
    nt_rrSetpoint = table->GetDoubleTopic("rrSetpoint").Publish();
    nt_rrVelocity = table->GetDoubleTopic("rrVelocity").Publish();
  }
  
  frc2::RobotModeTriggers::Test().OnTrue(frc2::InstantCommand([this]() {this->TestInit();}).ToPtr());
}

void DriveSubsystem::Periodic() {
  poseEstimator.Update(ahrs.GetRotation3d(), getWheelPositions());

  nt_flVelocity.Set(frontLeftMotor.GetEncoder().GetVelocity());
  nt_flOutput.Set(frontLeftMotor.GetAppliedOutput());

  nt_frVelocity.Set(frontRightMotor.GetEncoder().GetVelocity());
  nt_frOutput.Set(frontLeftMotor.GetAppliedOutput());

  nt_rlVelocity.Set(rearLeftMotor.GetEncoder().GetVelocity());
  nt_rlOutput.Set(rearLeftMotor.GetAppliedOutput());

  nt_rrVelocity.Set(rearRightMotor.GetEncoder().GetVelocity());;
  nt_rrOutput.Set(rearRightMotor.GetAppliedOutput());
}

void DriveSubsystem::TestInit() {
  frc::SmartDashboard::PutData("Drive/wheelSpeedTuner", &wheelSpeedTuner);
}

void DriveSubsystem::drive(frc::ChassisSpeeds speed, bool fieldRelative) {
  if (fieldRelative) {
    frc::Rotation2d angle = poseEstimator.GetEstimatedPosition().ToPose2d().Rotation();
    speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speed, angle);
  }
  frc::MecanumDriveWheelSpeeds wheelSpeeds = driveKinematics.ToWheelSpeeds(speed);

  frontRightMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.frontRight.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.frontRight).value()
  );
  nt_frSetpoint.Set(wheelSpeeds.frontRight.value());
  
  frontLeftMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.frontLeft.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.frontLeft).value()
  );
  nt_flSetpoint.Set(wheelSpeeds.frontLeft.value());
  
  rearRightMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.rearRight.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.rearRight).value()
  );
  nt_rrSetpoint.Set(wheelSpeeds.rearRight.value());
    
  rearLeftMotor.GetClosedLoopController().SetReference(
    wheelSpeeds.rearLeft.value(),
    SparkMax::ControlType::kVelocity,
    {},
    (driveVff * wheelSpeeds.rearLeft).value()
  );
  nt_rlSetpoint.Set(wheelSpeeds.rearLeft.value());
}

void DriveSubsystem::stop() {
  frontLeftMotor.StopMotor();
  frontRightMotor.StopMotor();
  rearLeftMotor.StopMotor();
  rearRightMotor.StopMotor();
}

void DriveSubsystem::resetFieldOrientation(bool inverted) {
  poseEstimator.ResetRotation(frc::Rotation3d(inverted ? frc::Rotation2d(180_deg) : frc::Rotation2d(0_deg)));
}

void DriveSubsystem::resetPose(frc::Pose3d pose) {
  poseEstimator.ResetPose(pose);
}

frc::Pose3d DriveSubsystem::getPose() {
  return poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::updateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp) {
  poseEstimator.AddVisionMeasurement(measurement, timestamp);
}

const frc::MecanumDriveWheelPositions DriveSubsystem::getWheelPositions() {
  return frc::MecanumDriveWheelPositions {
    units::meter_t{frontLeftMotor.GetEncoder().GetPosition()},
    units::meter_t{frontRightMotor.GetEncoder().GetPosition()},
    units::meter_t{rearLeftMotor.GetEncoder().GetPosition()},
    units::meter_t{rearRightMotor.GetEncoder().GetPosition()},
  };
}
