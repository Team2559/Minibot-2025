#pragma once

#include <choreo/Choreo.h>
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

/**
 * Command to follow a Choreo swerve trajectory
 *
 * This command starts by resetting the robot pose to the initial trajectory
 * pose, then follows the trajectory until the expected time has elapsed.
 */
class SwerveTrajectoryCommand : public frc2::CommandHelper<frc2::Command, SwerveTrajectoryCommand> {
  public:
    /**
     * Creates a new SwerveTrajectoryCommand
     *
     * @param subsystem The holonomic drivetrain subsystem to use
     * @param trajectory The trajectory to follow
     */
    explicit SwerveTrajectoryCommand(DriveSubsystem &subsystem, choreo::Trajectory<choreo::SwerveSample> &trajectory);

    void Initialize() override;

    void Execute() override;

    void End(bool wasCanceled) override;

    bool IsFinished() override;

  private:
    DriveSubsystem &m_driveSubsystem;
    choreo::Trajectory<choreo::SwerveSample> &m_trajectory;

    bool m_invertForRed = false;
    frc::Timer m_timer;
};
