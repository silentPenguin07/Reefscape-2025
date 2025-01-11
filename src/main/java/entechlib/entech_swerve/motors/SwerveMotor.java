package entechlib.entech_swerve.motors;

import entechlib.entech_swerve.ConfigConstructionUtil.ControlType;

/**
 * Basic interface for swerve motor for turning or driving.
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public interface SwerveMotor {
    /**
     * Set the values for the pid controller.
     * 
     * 
     * @param p
     * @param i
     * @param d
     * @param ff
     */
    void setPID(double p, double i, double d, double ff);

    /**
     * Set the method for controlling the motor with the pid.
     * 
     * 
     * @param control method
     */
    void setControlMethod(ControlType control);

    /**
     * Set the current limit for the motor.
     * 
     * 
     * @param limit
     */
    void setCurrentLimit(int limit);

    /**
     * Complete the configuration of the motor, not always necessary but highly
     * recommended to save settings after brownouts.
     */
    void completeConfigure();

    /**
     * Set the ratio of turns on the motors output to its start.
     * 
     * 
     * @param positionConversionFactor
     */
    void setPositionConversionFactor(double positionConversionFactor);

    /**
     * Set the ratio of velocity on the motors output to its start.
     * 
     * 
     * @param positionConversionFactor
     */
    void setVelocityConversionFactor(double positionConversionFactor);

    void set(double speed);

    void setPosition(double position);

    double getPosition();

    double getVelocity();

    void setReference(double value);

    boolean getInverted();

    void setInverted(boolean inverted);
}