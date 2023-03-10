package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    SwerveModuleState getState();

}