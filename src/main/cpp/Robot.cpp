#include "Robot.hpp"
#include "Drivetrain.hpp"
#include "Buttons.hpp"
#include "RobotState.hpp"

#include <frc/MathUtil.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

Robot::Robot()
{
  // setup RobotStates
  RobotState::IsEnabled = [this]()
  { return IsEnabled(); };
  RobotState::IsDisabled = [this]()
  { return IsDisabled(); };
  RobotState::IsAutonomous = [this]()
  { return IsAutonomous(); };
  RobotState::IsAutonomousEnabled = [this]()
  { return IsAutonomousEnabled(); };
  RobotState::IsTeleop = [this]()
  { return IsTeleop(); };
  RobotState::IsTeleopEnabled = [this]()
  { return IsTeleopEnabled(); };
  RobotState::IsTest = [this]()
  { return IsTest(); };

  Drivetrain::init();
}

void Robot::AutonomousPeriodic()
{
  // driveWithJoystick(false);
  Drivetrain::updateOdometry();
}

void Robot::TeleopPeriodic() { driveWithJoystick(true); }

void Robot::driveWithJoystick(bool field_relative)
{
  // Get the x speed.
  printf("Printing PS5 Inputs: X: %f, Y: %f, Z: %f\n", BUTTON::ps5.GetX(), BUTTON::ps5.GetY(), BUTTON::ps5.GetZ());
  auto const x_speed = frc::ApplyDeadband(BUTTON::ps5.GetX(), 0.04) * Drivetrain::K_MAX_SPEED;

  auto const y_speed = -frc::ApplyDeadband(BUTTON::ps5.GetY(), 0.04) * Drivetrain::K_MAX_SPEED;
  auto const rot = frc::ApplyDeadband(BUTTON::ps5.GetZ(), 0.04) * Drivetrain::K_MAX_ANGULAR_SPEED; // Might need to be inverted in the future
  Drivetrain::drive(x_speed, y_speed, rot, field_relative);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif