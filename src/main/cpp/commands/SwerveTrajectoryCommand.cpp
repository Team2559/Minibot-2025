#include "commands/SwerveTrajectoryCommand.h"

#include <frc/DriverStation.h>

SwerveTrajectoryCommand::SwerveTrajectoryCommand(DriveSubsystem &driveSubsystem, choreo::Trajectory<choreo::SwerveSample> &trajectory) :
    m_driveSubsystem{driveSubsystem},
    m_trajectory{trajectory},
    m_timer{} {}

void SwerveTrajectoryCommand::Initialize() {
  // Check which alliance we are on to determine what side of the field to run the trajectory on
  m_invertForRed = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed;
  // Reset the timer that tracks our progress through the trajectory
  m_timer.Reset();
  // Reset the robot pose to the starting pose of the trajectory
  std::optional<frc::Pose2d> initialPose = m_trajectory.GetInitialPose(m_invertForRed);
  if (initialPose.has_value()) {
    m_driveSubsystem.ResetPose(frc::Pose3d(initialPose.value()));
  }
  // Update the field display on the dashboard to show the expected trajectory path
}

void SwerveTrajectoryCommand::Execute() {
  // Sample a pose from the trajectory, then attempt to Drive to that pose
  std::optional<choreo::SwerveSample> sample = m_trajectory.SampleAt(m_timer.Get(), m_invertForRed);
  if (sample.has_value()) {
    m_driveSubsystem.FollowTrajectory(sample.value());
  }
}

void SwerveTrajectoryCommand::End(bool wasCanceled) {
  // Stop the drivetrain and clear the trajectory from the dashboard field
  m_driveSubsystem.Stop();
  m_timer.Stop();
}

bool SwerveTrajectoryCommand::IsFinished() {
  // End if the expected trajectory duration has elapsed
  return m_timer.Get() >= m_trajectory.GetTotalTime();
}