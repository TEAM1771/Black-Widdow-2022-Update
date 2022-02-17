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
        auto const robot_speeds = Trajectory::getEstimatedSpeeds();

        auto const camera_deg = units::degree_t{camera.getX()} + CAMERA_X_OFFSET;

        double const turret_pos_ticks = turretTurnyTurny.encoder.GetPosition();

        auto target_ticks = turret_pos_ticks + (camera_deg.value() * TICKS_PER_DEGREE);

        if (robot_speeds.vx > 0.3_mps || robot_speeds.vy > 0.3_mps)
        {
            // std::complex uses radians
            units::radian_t const target_rad{target_ticks / TICKS_PER_RADIAN};

            // y_speed reversed so left=negative and right=positive
            auto const robot_leftright_speed = -robot_speeds.vy;

            // Logic is copied from ChassisSpeeds::FromFieldRelativeSeeds
            auto const target_rel_leftright_speed = -robot_speeds.vx * sin(target_rad.value()) + robot_leftright_speed * cos(target_rad.value());

            // Uses atan2 to prevent divide by 0 warnings
            auto const robot_speed_argument = target_rad.value() - atan2(target_rel_leftright_speed.value(), 0);
            auto const robot_speed_magnitude = std::abs(target_rel_leftright_speed.value());

            std::complex<double> const robot_movement_vector = std::polar(robot_speed_magnitude, robot_speed_argument);

            // Speed of shooter wheel is ~20m/s
            auto const shoot_vector = std::polar(20.0, target_rad.value());

            auto const ball_vector = shoot_vector - robot_movement_vector;

            // std::arg returns angle in radians
            target_ticks = std::arg(ball_vector) * TICKS_PER_RADIAN;
        }

        static auto prev_offset_deg = 0_deg;
        if (prev_offset_deg == camera_deg) // Prevents reusing outdated camera data
            return {true, std::abs(camera_deg.value()) < tolerance};
        prev_offset_deg = camera_deg;

        turretTurnyTurny.SetTarget(target_ticks);

        return {true, std::abs(camera_deg.value()) < tolerance};
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
