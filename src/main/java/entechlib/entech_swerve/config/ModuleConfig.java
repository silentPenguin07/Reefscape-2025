package entechlib.entech_swerve.config;

/**
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public class ModuleConfig {
    private final int turningMotorID;
    private final int driveMotorID;
    private final int absoluteEncoderID;
    private double encoderOffsetRadians = 0;
    private boolean drivingMotorInverted = false;
    private boolean turningMotorInverted = true;

    /**
     * @param turningMotorID
     * @param driveMotorID
     * @param absoluteEncoderID use the analog port instead of canbus id if
     *                          needed.
     */
    public ModuleConfig(int turningMotorID,
            int driveMotorID, int absoluteEncoderID) {
        this.turningMotorID = turningMotorID;
        this.driveMotorID = driveMotorID;
        this.absoluteEncoderID = absoluteEncoderID;
    }

    public int getTurningMotorID() {
        return this.turningMotorID;
    }

    public int getDriveMotorID() {
        return this.driveMotorID;
    }

    public int getAbsoluteEncoderID() {
        return this.absoluteEncoderID;
    }

    public double getEncoderOffsetRadians() {
        return this.encoderOffsetRadians;
    }

    public void setEncoderOffsetRadians(double encoderOffsetRadians) {
        this.encoderOffsetRadians = encoderOffsetRadians;
    }

    public boolean isDrivingMotorInverted() {
        return this.drivingMotorInverted;
    }

    public boolean getDrivingMotorInverted() {
        return this.drivingMotorInverted;
    }

    public void setDrivingMotorInverted(boolean drivingMotorInverted) {
        this.drivingMotorInverted = drivingMotorInverted;
    }

    public boolean isTurningMotorInverted() {
        return this.turningMotorInverted;
    }

    public boolean getTurningMotorInverted() {
        return this.turningMotorInverted;
    }

    public void setTurningMotorInverted(boolean turningMotorInverted) {
        this.turningMotorInverted = turningMotorInverted;
    }
}