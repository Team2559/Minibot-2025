// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {
  inline constexpr int kDriverControllerPort = 0;
} // namespace OperatorConstants

namespace DriveConstants {
  inline constexpr int kFrontLeftDriveID = 0;
  inline constexpr int kFrontRightDriveID = 1;
  inline constexpr int kRearLeftDriveID = 69;
  inline constexpr int kRearRightDriveID = 67;

  inline constexpr double kDriveGearRatio = 4.0 / 1.0; // approx 5.90

  // Distance between the wheels, width and length
  constexpr units::meter_t kDriveBaseWidth = 19.8_in;
  constexpr units::meter_t kDriveBaseLength = 17.75_in;

  constexpr frc::Translation2d kFrontLeftWheel{kDriveBaseLength / 2.0, kDriveBaseWidth / 2.0};
  constexpr frc::Translation2d kFrontRightWheel{kDriveBaseLength / 2.0, -kDriveBaseWidth / 2.0};
  constexpr frc::Translation2d kBackLeftWheel{-kDriveBaseLength / 2.0, kDriveBaseWidth / 2.0};
  constexpr frc::Translation2d kBackRightWheel{-kDriveBaseLength / 2.0, -kDriveBaseWidth / 2.0};

  // This should be empirically determined!  This is just an initial guess.
  // This is used for both distance and velocity control. If this is off, it
  // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
  inline constexpr units::unit_t<units::compound_unit<units::meter, units::inverse<units::turn>>> kDriveDistancePerRotation =
    4.00_in * units::constants::pi / units::turn_t{kDriveGearRatio};

  constexpr units::meters_per_second_t kMaxDriveSpeed = 5676_rpm * kDriveDistancePerRotation;
  constexpr double kSlowDrivePercent = 0.80;

  // This is used for rotating the robot in place, about it's center.  This
  // may need to be empirically adjusted, but check kDriveMetersPerRotation
  // before making any adjustment here.
  constexpr units::meter_t kDriveMetersPerSteerCircle =
    M_PI * units::math::sqrt(units::math::pow<2>(kDriveBaseLength) + units::math::pow<2>(kDriveBaseWidth));

  constexpr units::radians_per_second_t kMaxTurnSpeed = kMaxDriveSpeed / kDriveMetersPerSteerCircle * 360_deg;

  constexpr frc::DCMotor driveMotorModel = frc::DCMotor::NEO();

  // Closed loop feedback for drive wheel velocities
  namespace DrivePID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
    constexpr double kV =
      (1.0 / driveMotorModel.Kv / kDriveDistancePerRotation)
        .convert<units::compound_unit<units::volt, units::inverse<units::meters_per_second>>>()
        .value();
  } // namespace DrivePID

  // Closed loop feedback for chassis translation
  namespace TranslationPID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
  } // namespace TranslationPID

  // Closed loop feedback for chassis orientation
  namespace OrientationPID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
  } // namespace OrientationPID
} // namespace DriveConstants

namespace VisionConstants {
  inline const frc::AprilTagFieldLayout kTagLayout{
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};
  inline const frc::Transform3d kRobotToCam{
    frc::Translation3d{0.5_m, 0.0_m, 0.5_m},
    frc::Rotation3d{0_rad, -30_deg, 0_rad}};
}
