#include "Turret.hpp"
//#include "PhotonVision.hpp"
#include "LimeLight.hpp"
#include "ngr.hpp"
#include "PID_CANSparkMax.hpp"
#include "Trajectory.hpp"

#include <wpi/numbers>
#include <units/angle.h>
#include <complex>

#include <cmath>

/******************************************************************/
/*                        Private Constants                        */
/******************************************************************/

constexpr units::degree_t CAMERA_X_OFFSET{3.75}; // 4.2517710;

constexpr int PORT = 6;

constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

// TOLERANCE in .hpp

constexpr double TICKS_PER_REVOLUTION = 212;
constexpr double TICKS_PER_RADIAN = TICKS_PER_REVOLUTION / (2 * wpi::numbers::pi);
constexpr double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360;

constexpr double TRAVERSE_SPEED = .7;

constexpr double P = 0.1;
constexpr double I = 0.0;
constexpr double D = 0.0;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

extern LimeLight camera; // camera from Robot.cpp

static PID_CANSparkMax turretTurnyTurny{PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static Turret::POSITION position = Turret::POSITION::ZERO;
static bool tracking = false;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

void Turret::init()
{
    turretTurnyTurny.RestoreFactoryDefaults();

    turretTurnyTurny.SetIdleMode(IDLE_MODE);

    turretTurnyTurny.SetSmartCurrentLimit(20);

    turretTurnyTurny.SetPID(P, I, D);
    turretTurnyTurny.SetOutputRange(-TRAVERSE_SPEED, TRAVERSE_SPEED);
    turretTurnyTurny.SetPositionRange(Turret::POSITION::MAX_LEFT, Turret::POSITION::MAX_RIGHT);
    turretTurnyTurny.SetTarget(Turret::POSITION::ZERO);
}

bool Turret::goToPosition(Turret::POSITION const &pos, double const &tolerance)
{
    if (pos != position)
    {
        turretTurnyTurny.SetTarget(pos);
        position = pos;
    }

    tracking = false; // Reset for Turret::visionTrack(...)

    return std::abs(turretTurnyTurny.encoder.GetPosition() - pos) < tolerance;
}

Turret::visionState Turret::visionTrack(Turret::POSITION const &initPosition, double const &tolerance)
{
    if (!tracking) // move to initPosition
    {
        tracking = goToPosition(initPosition);
        return {false, false};
    }
    //
    // photonlib::PhotonPipelineResult result = camera.GetLatestResult();

    if (camera.hasTarget())
    {
        // auto const target = result.GetBestTarget();
        auto const [robot_x_speed, robot_y_speed, robot_omega_speed] = Trajectory::getEstimatedSpeeds();

        auto offset_deg = units::degree_t{camera.getX()} + CAMERA_X_OFFSET;

        double const turret_pos_ticks = turretTurnyTurny.encoder.GetPosition();

        if (robot_x_speed > 1_mps || robot_y_speed > 1_mps)
        {

            units::degree_t const turret_pos_deg{turret_pos_ticks / TICKS_PER_DEGREE};

            units::degree_t const target_deg = offset_deg + turret_pos_deg;

            auto const target_relative_speeds =
                frc::ChassisSpeeds::FromFieldRelativeSpeeds(robot_x_speed, robot_y_speed, robot_omega_speed,
                                                            target_deg);

            auto const target_relative_y_speed = target_relative_speeds.vy.value();
            units::radian_t const offset_rad = offset_deg;
            auto const target_relative_angle = offset_rad.value() - atan2(target_relative_y_speed, 0);
            
            std::complex<double> const target_relative_robot_speed =
                std::polar(target_relative_y_speed, 
               target_relative_angle);

            // Speed of shooter wheel is 53.46 m/s
            auto shoot_speed = std::polar(53.46 / 2, offset_rad.value());

            auto const ball_speed = shoot_speed - target_relative_robot_speed;

            offset_deg = units::radian_t{std::arg(ball_speed)};
        }

        double const target_ticks = turret_pos_ticks + (offset_deg.value() * TICKS_PER_DEGREE);

        static auto prev_offset_deg = 0_deg;
        if (prev_offset_deg == offset_deg) // prevents reusing outdated data
            return {true, std::abs(offset_deg.value()) < tolerance};
        prev_offset_deg = offset_deg;

        turretTurnyTurny.SetTarget(target_ticks);

        return {true, std::abs(offset_deg.value()) < tolerance};
    }

    return {false, false};
}

void Turret::manualPositionControl(double const &pos)
{
    turretTurnyTurny.SetTarget(ngr::scaleOutput(
                                   -1,
                                   1,
                                   POSITION::MAX_LEFT,
                                   POSITION::MAX_RIGHT,
                                   std::clamp(pos, -1.0, 1.0)),
                               rev::CANSparkMax::ControlType::kPosition);
}

double Turret::getTemp()
{
    return turretTurnyTurny.GetMotorTemperature();
}
