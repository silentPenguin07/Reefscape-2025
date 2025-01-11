package entechlib.entech_swerve.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import entechlib.entech_swerve.ConfigConstructionUtil;
import entechlib.entech_swerve.ConfigConstructionUtil.GyroType;

/**
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public class SwerveConfig {

    private boolean rateLimiting = false;
    private RateLimiterConfig rateLimiterConfig = null;

    private SwerveDriveKinematics driveKinematics;
    /** Default value is 4 */
    private double maxSpeedMetersPerSecond = 4.0;
    /** Default value is 2PI */
    private double maxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    private ModuleConfig frontLeft;
    private ModuleConfig frontRight;
    private ModuleConfig rearLeft;
    private ModuleConfig rearRight;
    private GyroType gyroType;
    private boolean gyroInverted;

    private ConfigConstructionUtil.AbsoluteEncoderType encoderType;
    private ConfigConstructionUtil.MotorType turningMotorType;
    private ConfigConstructionUtil.MotorType drivingMotorType;

    /** Default limit is 40 amps */
    private int driveMotorCurrentLimit = 40;
    /** Default limit is 20 amps */
    private int turningMotorCurrentLimit = 20;
    /** Default limit is 0.1016 meters */
    private double wheelDiameterMeters = 0.1016;

    private boolean turningEncoderInverted = true;

    private double driveMotorFreeRPS = 5676.0 / 60.0;

    private double driveMotorReduction = 6.74603175;

    private double turningMotorReduction = 21.4285714;

    private double drivingPositionConversionMetersPerRotation = (wheelDiameterMeters * Math.PI) / driveMotorReduction;
    private double drivingVelocityConversionMetersPerSecondPerRPM = ((wheelDiameterMeters * Math.PI)
            / driveMotorReduction) / 60;

    private double turningPositionConversionRadiansPerRotation = (2 * Math.PI) / turningMotorReduction;
    private double turningVelocityConversionRadiansPerSecondPerRPM = (2 * Math.PI) / turningMotorReduction / 60;

    private double driveProportional = 0.04;
    private double turningProportional = 1.0;
    private double driveFeedForward = 1 / (((wheelDiameterMeters * Math.PI) * driveMotorFreeRPS) / driveMotorReduction);
    private AutoConfig autoConfig;

    public void setModuleHardware(double driveMotorFreeRPM, double driveMotorReduction, double turningMotorReduction,
            double wheelDiameterMeters) {
        driveMotorFreeRPS = driveMotorFreeRPM / 60;
        this.driveMotorReduction = driveMotorReduction;
        this.turningMotorReduction = turningMotorReduction;
        this.wheelDiameterMeters = wheelDiameterMeters;

        drivingPositionConversionMetersPerRotation = (wheelDiameterMeters * Math.PI) / driveMotorReduction;

        drivingVelocityConversionMetersPerSecondPerRPM = ((wheelDiameterMeters * Math.PI)
                / driveMotorReduction) / 60;

        turningPositionConversionRadiansPerRotation = (2 * Math.PI) / turningMotorReduction;

        turningVelocityConversionRadiansPerSecondPerRPM = (2 * Math.PI) / turningMotorReduction / 60;

        driveFeedForward = 1 / (((wheelDiameterMeters * Math.PI) * driveMotorFreeRPS) / driveMotorReduction);
    }

    public RateLimiterConfig getRateLimiterConfig() {
        return rateLimiterConfig;
    }

    public void setRobotHardware(double wheelBaseMeters, double trackWidthMeters) {
        driveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBaseMeters / 2, trackWidthMeters / 2),
                new Translation2d(wheelBaseMeters / 2, -trackWidthMeters / 2),
                new Translation2d(-wheelBaseMeters / 2, trackWidthMeters / 2),
                new Translation2d(-wheelBaseMeters / 2, -trackWidthMeters / 2));
    }

    public AutoConfig getAutoConfig() {
        return this.autoConfig;
    }

    public void setAutoConfig(AutoConfig autoConfig) {
        this.autoConfig = autoConfig;
    }

    public boolean isRateLimiting() {
        return rateLimiting;
    }

    public void withRateLimiting(RateLimiterConfig rateLimiterConfig) {
        this.rateLimiterConfig = rateLimiterConfig;
        rateLimiting = true;
    }

    public SwerveDriveKinematics getDriveKinematics() {
        return this.driveKinematics;
    }

    public void setDriveKinematics(SwerveDriveKinematics driveKinematics) {
        this.driveKinematics = driveKinematics;
    }

    public double getMaxSpeedMetersPerSecond() {
        return this.maxSpeedMetersPerSecond;
    }

    public void setMaxSpeedMetersPerSecond(double maxSpeedMetersPerSecond) {
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    }

    public double getMaxAngularSpeedRadiansPerSecond() {
        return this.maxAngularSpeedRadiansPerSecond;
    }

    public void setMaxAngularSpeedRadiansPerSecond(double maxAngularSpeedRadiansPerSecond) {
        this.maxAngularSpeedRadiansPerSecond = maxAngularSpeedRadiansPerSecond;
    }

    public ModuleConfig getFrontLeft() {
        return this.frontLeft;
    }

    public void setFrontLeft(ModuleConfig frontLeft) {
        this.frontLeft = frontLeft;
    }

    public ModuleConfig getFrontRight() {
        return this.frontRight;
    }

    public void setFrontRight(ModuleConfig frontRight) {
        this.frontRight = frontRight;
    }

    public ModuleConfig getRearLeft() {
        return this.rearLeft;
    }

    public void setRearLeft(ModuleConfig rearLeft) {
        this.rearLeft = rearLeft;
    }

    public ModuleConfig getRearRight() {
        return this.rearRight;
    }

    public void setRearRight(ModuleConfig rearRight) {
        this.rearRight = rearRight;
    }

    public int getDriveMotorCurrentLimit() {
        return this.driveMotorCurrentLimit;
    }

    public void setDriveMotorCurrentLimit(int driveMotorCurrentLimit) {
        this.driveMotorCurrentLimit = driveMotorCurrentLimit;
    }

    public int getTurningMotorCurrentLimit() {
        return this.turningMotorCurrentLimit;
    }

    public void setTurningMotorCurrentLimit(int turningMotorCurrentLimit) {
        this.turningMotorCurrentLimit = turningMotorCurrentLimit;
    }

    public boolean isTurningEncoderInverted() {
        return this.turningEncoderInverted;
    }

    public boolean getTurningEncoderInverted() {
        return this.turningEncoderInverted;
    }

    public void setTurningEncoderInverted(boolean turningEncoderInverted) {
        this.turningEncoderInverted = turningEncoderInverted;
    }

    public double getDriveProportional() {
        return this.driveProportional;
    }

    public void setDriveProportional(double driveProportional) {
        this.driveProportional = driveProportional;
    }

    public double getTurningProportional() {
        return this.turningProportional;
    }

    public void setTurningProportional(double turningProportional) {
        this.turningProportional = turningProportional;
    }

    public double getDriveMotorFreeRPS() {
        return this.driveMotorFreeRPS;
    }

    public double getDriveMotorReduction() {
        return this.driveMotorReduction;
    }

    public double getTurningMotorReduction() {
        return this.turningMotorReduction;
    }

    public double getDrivingPositionConversionMetersPerRotation() {
        return this.drivingPositionConversionMetersPerRotation;
    }

    public double getDrivingVelocityConversionMetersPerSecondPerRPM() {
        return this.drivingVelocityConversionMetersPerSecondPerRPM;
    }

    public double getTurningPositionConversionRadiansPerRotation() {
        return this.turningPositionConversionRadiansPerRotation;
    }

    public double getTurningVelocityConversionRadiansPerSecondPerRPM() {
        return this.turningVelocityConversionRadiansPerSecondPerRPM;
    }

    public double getDriveFeedForward() {
        return this.driveFeedForward;
    }

    public ConfigConstructionUtil.AbsoluteEncoderType getEncoderType() {
        return this.encoderType;
    }

    public void setEncoderType(ConfigConstructionUtil.AbsoluteEncoderType encoderType) {
        this.encoderType = encoderType;
    }

    public GyroType getGyroType() {
        return this.gyroType;
    }

    public void setGyroType(GyroType gyroType) {
        this.gyroType = gyroType;
    }

    public boolean isGyroInverted() {
        return this.gyroInverted;
    }

    public boolean getGyroInverted() {
        return this.gyroInverted;
    }

    public void setGyroInverted(boolean gyroInverted) {
        this.gyroInverted = gyroInverted;
    }

    public ConfigConstructionUtil.MotorType getTurningMotorType() {
        return this.turningMotorType;
    }

    public void setTurningMotorType(ConfigConstructionUtil.MotorType turningMotorType) {
        this.turningMotorType = turningMotorType;
    }

    public ConfigConstructionUtil.MotorType getDrivingMotorType() {
        return this.drivingMotorType;
    }

    public void setDrivingMotorType(ConfigConstructionUtil.MotorType drivingMotorType) {
        this.drivingMotorType = drivingMotorType;
    }
}
