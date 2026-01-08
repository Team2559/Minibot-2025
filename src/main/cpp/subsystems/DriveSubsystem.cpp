// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace DriveConstants;
using namespace rev::spark;

DriveSubsystem::DriveSubsystem() :
    frontLeftMotor{kFrontLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
    frontRightMotor{kFrontRightDriveID, rev::spark::SparkMax::MotorType::kBrushless},
    rearLeftMotor{kRearLeftDriveID, rev::spark::SparkMax::MotorType::kBrushless},
    rearRightMotor{kRearRightDriveID, rev::spark::SparkMax::MotorType::kBrushless},
    ahrs{studica::AHRS::NavXComType::kMXP_SPI},
    poseEstimator{driveKinematics, ahrs.GetRotation3d(), GetWheelPositions(), frc::Pose3d()},
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
    }},
    xController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    yController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    rController{OrientationPID::kP, OrientationPID::kI, OrientationPID::kD} {
  {
    // Drive motor configuration

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
    // NetworkTables logging initialization

    auto ntInstance = nt::NetworkTableInstance::GetDefault();
    auto table = ntInstance.GetTable("Drive");

    nt_flVelocity = table->GetDoubleTopic("flVelocity").Publish();
    nt_flSetpoint = table->GetDoubleTopic("flSetpoint").Publish();
    nt_flOutput = table->GetDoubleTopic("flOutput").Publish();

    nt_frVelocity = table->GetDoubleTopic("frVelocity").Publish();
    nt_frSetpoint = table->GetDoubleTopic("frSetpoint").Publish();
    nt_frOutput = table->GetDoubleTopic("frOutput").Publish();

    nt_rlVelocity = table->GetDoubleTopic("rlVelocity").Publish();
    nt_rlSetpoint = table->GetDoubleTopic("rlSetpoint").Publish();
    nt_rlOutput = table->GetDoubleTopic("rlOutput").Publish();

    nt_rrVelocity = table->GetDoubleTopic("rrVelocity").Publish();
    nt_rrSetpoint = table->GetDoubleTopic("rrSetpoint").Publish();
    nt_rrOutput = table->GetDoubleTopic("rrOutput").Publish();

    nt_xPosition = table->GetDoubleTopic("xPosition").Publish();
    nt_xOutput = table->GetDoubleTopic("xOutput").Publish();
    nt_xSetpoint = table->GetDoubleTopic("xSetpoint").Publish();

    nt_yOutput = table->GetDoubleTopic("yOutput").Publish();
    nt_yPosition = table->GetDoubleTopic("yPosition").Publish();
    nt_ySetpoint = table->GetDoubleTopic("ySetpoint").Publish();

    nt_rPosition = table->GetDoubleTopic("rPosition").Publish();
    nt_rSetpoint = table->GetDoubleTopic("rSetpoint").Publish();
    nt_rOutput = table->GetDoubleTopic("rOutput").Publish();
  }

  frc2::RobotModeTriggers::Test().OnTrue(frc2::InstantCommand([this]() { this->TestInit(); }).ToPtr());
}

void DriveSubsystem::Periodic() {
  frc::Pose2d pose = poseEstimator.Update(ahrs.GetRotation3d(), GetWheelPositions()).ToPose2d();

  nt_flVelocity.Set(frontLeftMotor.GetEncoder().GetVelocity());
  nt_flOutput.Set(frontLeftMotor.GetAppliedOutput());

  nt_frVelocity.Set(frontRightMotor.GetEncoder().GetVelocity());
  nt_frOutput.Set(frontLeftMotor.GetAppliedOutput());

  nt_rlVelocity.Set(rearLeftMotor.GetEncoder().GetVelocity());
  nt_rlOutput.Set(rearLeftMotor.GetAppliedOutput());

  nt_rrVelocity.Set(rearRightMotor.GetEncoder().GetVelocity());
  nt_rrOutput.Set(rearRightMotor.GetAppliedOutput());

  nt_xPosition.Set(pose.X().value());
  nt_yPosition.Set(pose.Y().value());
  nt_rPosition.Set(pose.Rotation().Radians().value());
}

void DriveSubsystem::TestInit() {
  frc::SmartDashboard::PutData("Drive/wheelSpeedTuner", &wheelSpeedTuner);
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speed, bool fieldRelative) {
  nt_xOutput.Set(speed.vx.value());
  nt_yOutput.Set(speed.vy.value());
  nt_rOutput.Set(speed.omega.value());

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

void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample &sample) {
  frc::Pose2d pose = GetPose().ToPose2d();

  nt_xSetpoint.Set(sample.x.value());
  nt_ySetpoint.Set(sample.y.value());
  nt_rSetpoint.Set(sample.heading.value());

  units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
  units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};

  units::radian_t robot_heading = pose.Rotation().Radians();
  units::radian_t sample_heading = sample.heading;
  units::radian_t target_heading = frc::AngleModulus(sample_heading - robot_heading) + robot_heading;

  units::radians_per_second_t rFeedback{rController.Calculate(robot_heading.value(), target_heading.value())};

  Drive(
    frc::ChassisSpeeds(
      sample.vx + xFeedback,
      sample.vy + yFeedback,
      sample.omega + rFeedback
    ),
    true
  );
}

void DriveSubsystem::Stop() {
  frontLeftMotor.StopMotor();
  frontRightMotor.StopMotor();
  rearLeftMotor.StopMotor();
  rearRightMotor.StopMotor();
}

void DriveSubsystem::ResetFieldOrientation(bool inverted) {
  poseEstimator.ResetRotation(frc::Rotation3d(inverted ? frc::Rotation2d(180_deg) : frc::Rotation2d(0_deg)));
}

void DriveSubsystem::ResetPose(frc::Pose3d pose) {
  poseEstimator.ResetPose(pose);
}

frc::Pose3d DriveSubsystem::GetPose() {
  return poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::UpdateVisionPose(frc::Pose3d measurement, units::millisecond_t timestamp) {
  poseEstimator.AddVisionMeasurement(measurement, timestamp);
}

const frc::MecanumDriveWheelPositions DriveSubsystem::GetWheelPositions() {
  return frc::MecanumDriveWheelPositions{
    units::meter_t{frontLeftMotor.GetEncoder().GetPosition()},
    units::meter_t{frontRightMotor.GetEncoder().GetPosition()},
    units::meter_t{rearLeftMotor.GetEncoder().GetPosition()},
    units::meter_t{rearRightMotor.GetEncoder().GetPosition()},
  };
}
