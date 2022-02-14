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
static rev::SparkMaxRelativeEncoder encoder;

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
    encoder = turretTurnyTurny.GetEncoder();
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

        auto x_offset_deg = units::degree_t{camera.getX()} + CAMERA_X_OFFSET;

        if (robot_x_speed > 1_mps || robot_y_speed > 1_mps)
        {

            // Add position conversion ratio to get to degrees
            frc::Rotation2d const turret_angle = units::degree_t{encoder.GetPosition()};

            auto const [turret_relative_front_back_robot_speed, turret_relative_left_right_robot_speed, _] =
                frc::ChassisSpeeds::FromFieldRelativeSpeeds(robot_x_speed, robot_y_speed, robot_omega_speed, turret_angle);

            // From here on, x = horizontal (left/right), y = verticle (front/back)

            std::complex<double> const robot_speed{turret_relative_left_right_robot_speed.value(), turret_relative_front_back_robot_speed.value()};

            auto const rotations_per_second = 6700 / 60;
            units::radians_per_second_t const radians_per_second{rotations_per_second / (2 * wpi::numbers::pi)};
            units::meters_per_second_t const meters_per_second{radians_per_second.value() * units::meter_t{3_in}.value()};
           
            auto shoot_speed = std::polar(meters_per_second.value() / 2, units::radian_t{x_offset_deg}.value());

            auto const ball_speed = shoot_speed - robot_speed;

            // unit circle angle converted to being front-centered, CW
            units::degree_t const movement_angle = units::radian_t{std::arg(ball_speed)};

            x_offset_deg = units::radian_t{std::arg(ball_speed)};
        }
        double const x_offset_ticks = x_offset_deg.value() * TICKS_PER_DEGREE;

        double const x_pos = turretTurnyTurny.encoder.GetPosition();
        double const x_target = x_pos + x_offset_ticks;

        static auto prev_offset_deg = 0_deg;
        if (prev_offset_deg == x_offset_deg) // prevents reusing outdated data
            return {true, std::abs(x_offset_deg.value()) < tolerance};
        prev_offset_deg = x_offset_deg;

        turretTurnyTurny.SetTarget(x_target);

        return {true, std::abs(x_offset_deg.value()) < tolerance};
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
