// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "commands/SwerveTrajectoryCommand.h"

auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ExampleTrajectory");

frc2::CommandPtr autos::FallbackAuto(DriveSubsystem &driveSubsystem) {
  return frc2::RunCommand([&]() {
           driveSubsystem.Drive(
             frc::ChassisSpeeds(
               0.67_mps,
               0.0_mps,
               0.0_rad_per_s
             ),
             false
           );
         })
    .ToPtr();
}

frc2::CommandPtr autos::ExampleAuto(DriveSubsystem &driveSubsystem) {
  if (trajectory.has_value()) {
    return SwerveTrajectoryCommand(driveSubsystem, trajectory.value()).ToPtr();
  } else {
    return FallbackAuto(driveSubsystem);
  }
}
