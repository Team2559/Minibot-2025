// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
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

namespace MotorConstants {
  // Speed constant for a Neo Vortex, in turns per second per volt
  constexpr units::unit_t<units::compound_unit<units::turns_per_second, units::inverse<units::volt>>> kVNeoVortex = 5676_rpm / 12.0_V;
  // Speed constant for a Neo 1.0, in turns per second per volt
  constexpr units::unit_t<units::compound_unit<units::turns_per_second, units::inverse<units::volt>>> kVNeo1 = 5676_rpm / 12.0_V;
  // Speed constant for a REV Neo 550, in turns per second per volt
  constexpr units::unit_t<units::compound_unit<units::turns_per_second, units::inverse<units::volt>>> kVNeo550 = 917.0_rpm / 1.0_V;
  // Torque constant for a REV Neo 550, in turns newton-meters per amp
  constexpr units::unit_t<units::compound_unit<units::newton_meter, units::inverse<units::ampere>>> kTNeo550 = 1.0_rad / kVNeo550;
  // Torque constant for a CTR Minion, in newton-meters per amp
  constexpr units::unit_t<units::compound_unit<units::newton_meter, units::inverse<units::ampere>>> kTMinion = 0.01568_Nm / 1.0_A;
  // Speed constant for a CTR Minion, in turns per second per volt
  constexpr units::unit_t<units::compound_unit<units::turns_per_second, units::inverse<units::volt>>> kVMinion = 1.0_rad / kTMinion;
}

namespace OperatorConstants {

  inline constexpr int kDriverControllerPort = 0;

}

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
  inline constexpr units::unit_t<units::compound_unit<units::meter, units::inverse<units::turn>>> kDriveDistancePerRotation = 4.00_in * units::constants::pi / units::turn_t{kDriveGearRatio};

  constexpr units::meters_per_second_t kMaxDriveSpeed = 5676_rpm * kDriveDistancePerRotation;
  constexpr double kSlowDrivePercent = 0.80;

  // This is used for rotating the robot in place, about it's center.  This
  // may need to be empirically adjusted, but check kDriveMetersPerRotation
  // before making any adjustment here.
  const units::meter_t kDriveMetersPerSteerCircle = M_PI * units::math::sqrt(units::math::pow<2>(kDriveBaseLength) + units::math::pow<2>(kDriveBaseWidth));

  const units::radians_per_second_t kMaxTurnSpeed = kMaxDriveSpeed / kDriveMetersPerSteerCircle * 360_deg;

  // Closed loop feedback for drive wheel velocities
  namespace DrivePID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
    constexpr double kV = (1.0 / MotorConstants::kVNeo1 / kDriveDistancePerRotation).value();
  }

  // Closed loop feedback for chassis translation
  namespace TranslationPID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
  }

  // Closed loop feedback for chassis orientation
  namespace OrientationPID {
    inline constexpr double kP = 1.0;
    inline constexpr double kI = 0.0;
    inline constexpr double kD = 0.0;
  }
}