#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>

#include <ctre/Phoenix.h>

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(int driver_adr, int turner_adr, int cancoder_adr, units::meter_t pos_x, units::meter_t pos_y);

    frc::SwerveModuleState getState();

    units::degree_t getAngle();

    void setDesiredState(const frc::SwerveModuleState &state);

    // Allows SwerveModule to be placed into Kinematics
    operator frc::Translation2d() const { return {pos_x, pos_y}; }

    // No copies/moves should be occuring
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

private:
    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    TalonFX driver, turner;
    CANCoder cancoder;
    units::meter_t pos_x;
    units::meter_t pos_y;
};
