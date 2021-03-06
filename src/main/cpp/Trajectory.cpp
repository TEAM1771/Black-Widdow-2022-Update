#include "Trajectory.hpp"
#include "Drivetrain.hpp"
#include "RobotState.hpp"
#include "ngr.hpp"
#include "Odometry.hpp"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>
#include <frc/Timer.h>


/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// This is using lambdas in order to use setters at beginning of runtime & save performance later
static frc::HolonomicDriveController controller{
    frc2::PIDController{1, 0, 0},
    frc2::PIDController{1, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        10, -0.003, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            Drivetrain::TRAJ_MAX_ANGULAR_SPEED,
            Drivetrain::TRAJ_MAX_ANGULAR_ACCELERATION}}};


/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Trajectory::printRobotRelativeSpeeds()
{
    frc::ChassisSpeeds const robot_relative = Drivetrain::getRobotRelativeSpeeds();

    frc::SmartDashboard::PutNumber("Estimated VX Speed", robot_relative.vx.value());
    frc::SmartDashboard::PutNumber("Estimated VY Speed", robot_relative.vy.value());
    frc::SmartDashboard::PutNumber("Estimated Omega Speed", units::degrees_per_second_t{robot_relative.omega}.value() / 720);
}

void Trajectory::printFieldRelativeSpeeds()
{
    frc::ChassisSpeeds const real_speeds = Odometry::getFieldRelativeSpeeds();

    frc::SmartDashboard::PutNumber("Real VX Speed", real_speeds.vx.value());
    frc::SmartDashboard::PutNumber("Real VY Speed", real_speeds.vy.value());
    frc::SmartDashboard::PutNumber("Real Omega Speed", units::degrees_per_second_t{real_speeds.omega}.value() / 720);
}

/******************************************************************/
/*                     Trajectory Functions                       */
/******************************************************************/

void Trajectory::driveToState(PathPlannerTrajectory::PathPlannerState const &state)
{
    // Correction to help the robot follow trajectory (combination of original trajectory speeds & error correction)
    frc::ChassisSpeeds const correction = controller.Calculate(Odometry::getPose(), state.pose, state.velocity, state.holonomicRotation);
    Drivetrain::faceDirection(correction.vx, correction.vy, state.holonomicRotation.Degrees(), false, 4, Drivetrain::TRAJ_MAX_ANGULAR_SPEED);

    if constexpr (debugging)
    {
        auto const real_pose = Odometry::getPose();
        frc::Transform2d const holonomic_error = {real_pose, state.pose};

        frc::SmartDashboard::PutNumber("Holonomic x error", holonomic_error.X().value());
        frc::SmartDashboard::PutNumber("Holonomic y error", holonomic_error.Y().value());
        frc::SmartDashboard::PutNumber("Holonomic z error", holonomic_error.Rotation().Radians().value());

        frc::SmartDashboard::PutNumber("Target Rotation", state.holonomicRotation.Radians().value());
        frc::SmartDashboard::PutNumber("Real Rotation", real_pose.Rotation().Radians().value());
    }
}

void Trajectory::follow(std::string const &traj_dir,
                        std::function<void(units::second_t time)> const &periodic,
                        units::meters_per_second_t const &max_vel,
                        units::meters_per_second_squared_t const &max_accl)
{
    auto traj = PathPlanner::loadPath(traj_dir, max_vel, max_accl, reverse_trajectory);

    auto const inital_state = *traj.getInitialState();
    auto const inital_pose = inital_state.pose;

    // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
    // to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
    Odometry::resetPosition({inital_pose.Translation(), inital_state.holonomicRotation}, Drivetrain::getCCWHeading());

    frc::Timer trajTimer;
    trajTimer.Start();

    if constexpr (debugging)
    {
        // If needed, we can disable the "error correction" for x & y
        controller.SetEnabled(true);

        frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));
    }

    while (RobotState::IsAutonomousEnabled() && (trajTimer.Get() <= traj.getTotalTime() + 0.1_s))
    {
        auto current_time = trajTimer.Get();

        auto sample = traj.sample(current_time);

        Odometry::getField2dObject("Traj")->SetPose({sample.pose.X(), sample.pose.Y(), sample.holonomicRotation});

        driveToState(sample);
        Odometry::update();

        if (periodic)
            periodic(current_time);

        if constexpr (debugging)
        {
            static int trajectory_samples{};
            frc::SmartDashboard::PutString("Sample:", fmt::format(
                                                          "Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
                                                          ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
                                                          sample.holonomicRotation.Degrees().value(), trajTimer.Get().value()));
            printRobotRelativeSpeeds();
            printFieldRelativeSpeeds();
        }

        using namespace std::chrono_literals;
        // This is the refresh rate of the HolonomicDriveController's PID controllers (can be tweaked if needed)
        std::this_thread::sleep_for(20ms);
    }
    Drivetrain::stop();
}

void Trajectory::testHolonomic(frc::Pose2d const &target_pose, units::velocity::meters_per_second_t const &velocity, frc::Rotation2d const &target_rot)
{
    Drivetrain::drive(controller.Calculate(Odometry::getPose(), target_pose, velocity, target_rot));
}