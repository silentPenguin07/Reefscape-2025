// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package entechlib.entech_swerve;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import entechlib.FieldConstants;
import entechlib.entech_swerve.config.AutoConfig;
import entechlib.entech_swerve.config.SwerveConfig;
import entechlib.entech_swerve.imus.navxMXP;
import entechlib.entech_swerve.imus.swerveIMU;
import entechlib.input.DriveInput;
import entechlib.math.RateLimiter;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the
 * function of the drivetrain.
 */
public class SwerveDrive {
    private final SwerveConfig swerveConfig;

    public static final int GYRO_ORIENTATION = 1; // might be able to merge with kGyroReversed
    private RateLimiter rateLimiter;

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;

    private swerveIMU gyro;

    private SwerveDriveOdometry odometry;

    Field2d field = new Field2d();

    private double getGyroAngle() {
        return gyro.getAngle() + 0;
    }

    public void periodic() {
        field.setRobotPose(odometry.getPoseMeters());

        odometry.update(
                Rotation2d.fromDegrees(GYRO_ORIENTATION * gyro.getAngle()),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(GYRO_ORIENTATION * gyro.getAngle()),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(DriveInput driveInput, boolean fieldRelative) {
        if (swerveConfig.isRateLimiting()) {
            driveInput = rateLimiter.limit(driveInput);
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = driveInput.getXSpeed() * swerveConfig.getMaxSpeedMetersPerSecond();
        double ySpeedDelivered = driveInput.getYSpeed() * swerveConfig.getMaxSpeedMetersPerSecond();
        double rotDelivered = driveInput.getRotationSpeed()
                * swerveConfig.getMaxAngularSpeedRadiansPerSecond();

        var swerveModuleStates = swerveConfig.getDriveKinematics().toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(GYRO_ORIENTATION * getGyroAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, swerveConfig.getMaxSpeedMetersPerSecond());

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void lockX() {
        frontLeft.setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0,
                Rotation2d.fromDegrees(45)));
    }

    /**
     * Resets the drive encoders to currently read a position of 0 and seeds the
     * turn encoders using the absolute encoders.
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleOffset(180);
        Pose2d pose = getPose();
        Pose2d pose2 = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(0));
        resetOdometry(pose2);
    }

    public Command driveTrajectoryCommand(Trajectory trajectory, Subsystem driveSubsystem) {
        AutoConfig config = swerveConfig.getAutoConfig();
        HolonomicDriveController controller = new HolonomicDriveController(config.getXController(),
                config.getYController(), config.getRotController());

        Consumer<SwerveModuleState[]> stateSetter = (SwerveModuleState[] swerveModuleStates) -> {
            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            rearLeft.setDesiredState(swerveModuleStates[2]);
            rearRight.setDesiredState(swerveModuleStates[3]);
        };
        return new SwerveControllerCommand(trajectory, this::getPose, swerveConfig.getDriveKinematics(), controller,
                stateSetter, driveSubsystem);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Double getHeading() {
        return Rotation2d.fromDegrees(GYRO_ORIENTATION * getGyroAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public Double getTurnRate() {
        return gyro.getRate() * (swerveConfig.getGyroInverted() ? -1.0 : 1.0);
    }

    public SwerveDrive(SwerveConfig swerveConfig) {
        this.swerveConfig = swerveConfig;

        frontLeft = new SwerveModule(swerveConfig, swerveConfig.getFrontLeft());
        frontRight = new SwerveModule(swerveConfig, swerveConfig.getFrontRight());
        rearLeft = new SwerveModule(swerveConfig, swerveConfig.getRearLeft());
        rearRight = new SwerveModule(swerveConfig, swerveConfig.getRearRight());

        gyro = new navxMXP();
        gyro.reset();
        gyro.zeroYaw();
        if (swerveConfig.isRateLimiting()) {
                rateLimiter = new RateLimiter(swerveConfig.getRateLimiterConfig());
        }

        odometry = new SwerveDriveOdometry(
                swerveConfig.getDriveKinematics(),
                Rotation2d.fromDegrees(GYRO_ORIENTATION * gyro.getAngle()),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                });

        resetEncoders();
        zeroHeading();

        Translation2d initialTranslation = new Translation2d(
                Units.inchesToMeters(FieldConstants.FIELD_LENGTH_INCHES / 2),
                Units.inchesToMeters(FieldConstants.FIELD_WIDTH_INCHES / 2)); // mid field
        Rotation2d initialRotation = Rotation2d.fromDegrees(180);
        gyro.setAngleOffset(0);
        Pose2d initialPose = new Pose2d(initialTranslation, initialRotation);
        resetOdometry(initialPose);
    }
}
