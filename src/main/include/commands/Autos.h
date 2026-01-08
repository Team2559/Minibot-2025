// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"

namespace autos {
  /**
   * Fallback auto to slowly drive one meter off line if loading the desired auto fails
   */
  frc2::CommandPtr FallbackAuto(DriveSubsystem &driveSubsystem);

  /**
   * Example static factory for a path-based autonomous command.
   */
  frc2::CommandPtr ExampleAuto(DriveSubsystem &driveSubsystem);
} // namespace autos
