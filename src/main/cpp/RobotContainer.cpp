// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

#include "ButtonUtil.h"
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  frc2::RobotModeTriggers::Disabled().OnFalse(
    frc2::InstantCommand([this]() {
      std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
      if (alliance.has_value()) {
        m_isRedAlliance = alliance.value() == frc::DriverStation::Alliance::kRed;
      }
    }).ToPtr()
  );

  driveSubsystem.SetDefaultCommand(
    frc2::RunCommand(
      [this]() {
        const auto controls = GetDriveTeleopControls();

        driveSubsystem.drive(
          frc::ChassisSpeeds(
            std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
            std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
            std::get<2>(controls) * DriveConstants::kMaxTurnSpeed
          ),
          std::get<3>(controls)
        );
      },
      {&driveSubsystem}
    )
  );

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_driverController.Back().OnTrue(
    frc2::InstantCommand([this]() {
      driveSubsystem.resetFieldOrientation(m_isRedAlliance);
    }).ToPtr()
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() {
  double boostTriggerValue = m_driverController.GetLeftTriggerAxis();
  double speedStickX = -m_driverController.GetLeftY();
  double speedStickY = -m_driverController.GetRightX();
  double headingStickY = -m_driverController.GetRightX();

  // Limit driving speed according to the current maximum speed
  if (boostTriggerValue < 0.5) {
    speedStickX *= DriveConstants::kSlowDrivePercent;
    speedStickY *= DriveConstants::kSlowDrivePercent;
  }

  // Flip field-relative velocities if the driver is on the red side of the field
  if (m_isRedAlliance) {
    speedStickX = -speedStickX;
    speedStickY = -speedStickY;
  }

  // Rescale final speeds to be more controllable
  speedStickX = ConditionRawJoystickInput(speedStickX);
  speedStickY = ConditionRawJoystickInput(speedStickY);
  headingStickY = ConditionRawJoystickInput(headingStickY);

  return std::make_tuple(speedStickX, speedStickY, headingStickY, m_fieldOriented);
}
