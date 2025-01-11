// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package entechlib.entech_swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import entechlib.entech_swerve.config.ModuleConfig;
import entechlib.entech_swerve.config.SwerveConfig;
import entechlib.entech_swerve.encoders.AbsoluteEncoder;
import entechlib.entech_swerve.motors.SwerveMotor;

/**
 * The {@code SwerveModule} class contains fields and methods pertaining to the
 * function of a swerve module.
 */
public class SwerveModule {
    private final SwerveMotor drivingMotor;
    private final SwerveMotor turningMotor;
    private final AbsoluteEncoder turningAbsoluteEncoder;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller.
     */
    public SwerveModule(SwerveConfig config, ModuleConfig moduleConfig) {
        drivingMotor = ConfigConstructionUtil.getTurningMotor(config, moduleConfig.getTurningMotorID(),
                        moduleConfig.getTurningMotorInverted());
        turningMotor = ConfigConstructionUtil.getDrivingMotor(config, moduleConfig.getTurningMotorID(),
                        moduleConfig.getDrivingMotorInverted());

        turningAbsoluteEncoder = ConfigConstructionUtil.getAbsoluteEncoder(config,
                        moduleConfig.getAbsoluteEncoderID(), moduleConfig.getEncoderOffsetRadians());

        desiredState.angle = new Rotation2d(turningAbsoluteEncoder.getPosition());
        drivingMotor.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingMotor.getVelocity(),
                        new Rotation2d(turningMotor.getPosition()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                        drivingMotor.getPosition(),
                        new Rotation2d(turningMotor.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                        new Rotation2d(turningMotor.getPosition()));

        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.001 // less than 1 mm per sec
                && Math.abs(optimizedDesiredState.angle.getRadians()
                                - turningMotor.getPosition()) < 0.1) // 10% of a radian
        {
            drivingMotor.set(0); // no point in doing anything
            turningMotor.set(0);
        } else {
            // Command driving and turning SPARKS MAX towards their respective setpoints.
            drivingMotor.setReference(optimizedDesiredState.speedMetersPerSecond);
            turningMotor.setReference(optimizedDesiredState.angle.getRadians());
        }

        this.desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule relative encoders. */
    public void resetEncoders() {
        drivingMotor.setPosition(0); // arbitrarily set driving encoder to zero

        turningMotor.set(0); // no moving during reset of relative turning encoder

        turningMotor.setPosition(turningAbsoluteEncoder.getVirtualPosition()); // set relative position
                                                                                // based on
                                                                                // virtual absolute
                                                                                // position
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }
}
