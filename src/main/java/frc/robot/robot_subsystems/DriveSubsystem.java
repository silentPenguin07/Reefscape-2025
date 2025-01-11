package frc.robot.robot_subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.I2C.Port;
import entechlib.entech_subsystems.EntechSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.robot_swerve.SwerveModule;
import frc.robot.robot_swerve.SwerveUtils;


public class DriveSubsystem extends EntechSubsystem{
    
    private static final boolean ENABLED = true;

    public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 2.32759093;
    public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 1.489458748; 
    public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 5.80897935;
    public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0.7840019;

    public static final int GYRO_ORIENTATION = -1; // might be able to merge with kGyroReversed

    public static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
    public static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in

    private Field2d field = new Field2d();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(0.4, 0.4),
        new Translation2d(0.4, 0.4),
        new Translation2d(0.4, 0.4),
        new Translation2d(0.4, 0.4)
    );
    
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_rearLeft;
    private SwerveModule m_rearRight;

    private AHRS m_gyro;

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private SwerveDriveOdometry m_odometry;

    // constructor
    public DriveSubsystem()
    {
        
        // autobuilder configuration
        AutoBuilder.configureHolonomic(
            this::getPose, // robot pose supplier
            this::resetOdometry, // resets odometry
            this::getSpeeds, // ChassisSpeeds supplier
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(.04, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                4.5, 
                Units.inchesToMeters(21.287), 
                new ReplanningConfig()),
            () -> {

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

        // set up custom logging to add the current path to a field 2d widget
        // TODO: figure out what this does
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    
            SmartDashboard.putData("Field", field);
    }

    private double getGyroAngle() {
        return m_gyro.getAngle() + 0;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     * public Optional<Pose2d> getPose() {
        return ENABLED ? Optional.of(m_odometry.getPoseMeters()) : Optional.empty();
    }
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    public Command auton() {
        return new PathPlannerAuto("Straight Auto");
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        if (ENABLED) {
            m_odometry.resetPosition(
                    Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                    },
                    pose);
        }
    }



    // returns the Module speeds in ChassisSpeeds form
    public ChassisSpeeds getSpeeds()
    {
        return kinematics.toChassisSpeeds(getModuleStates());
    }



    // helper method for returning an array of SwerveModuleState
    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = m_frontLeft.getState();
        states[1] = m_frontRight.getState();
        states[2] = m_rearLeft.getState();
        states[3] = m_rearRight.getState();

        return states;
    }

    // no idea what this is either
    // TODO: figure out what these methods even do
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds)
    {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    // no idea what this is yet
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds)
    {
        /*
         * TODO: Change the second parameter of ChassisSpeeds.discretize()
         * Parameters:
            continuousSpeeds The continuous speeds.
            dtSeconds The duration of the timestep the speeds should be applied for.
         */
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);

        setModuleStates(targetStates);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (ENABLED) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

            m_frontLeft.setDesiredState(desiredStates[0]);
            m_frontRight.setDesiredState(desiredStates[1]);
            m_rearLeft.setDesiredState(desiredStates[2]);
            m_rearRight.setDesiredState(desiredStates[3]);
        }
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
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        if (ENABLED) {
            double xSpeedCommanded;
            double ySpeedCommanded;

            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

                // Calculate the direction slew rate based on an estimate of the lateral
                // acceleration
                double directionSlewRate;

                if (m_currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
                }

                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - m_prevTime;
                double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

                if (angleDif < 0.45 * Math.PI) {
                    m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85 * Math.PI) {
                    if (m_currentTranslationMag > 1e-4) {
                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
                    } else {
                        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }

                m_prevTime = currentTime;

                xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
                ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
                m_currentRotation = m_rotLimiter.calculate(rot);

            } else {
                xSpeedCommanded = xSpeed;
                ySpeedCommanded = ySpeed;
                m_currentRotation = rot;
            }

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                    Rotation2d.fromDegrees(GYRO_ORIENTATION * getGyroAngle()))
                            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

            SwerveDriveKinematics.desaturateWheelSpeeds(
                    swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            m_rearLeft.setDesiredState(swerveModuleStates[2]);
            m_rearRight.setDesiredState(swerveModuleStates[3]);
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        if (ENABLED) {
            m_frontLeft.setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(45)));
            m_frontRight.setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-45)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(-45)));
            m_rearRight.setDesiredState(new SwerveModuleState(0,
                    Rotation2d.fromDegrees(45)));
        }
    }

    

    /**
     * Resets the drive encoders to currently read a position of 0 and seeds the
     * turn encoders using the absolute encoders.
     */
    public void resetEncoders() {
        if (ENABLED) {
            m_frontLeft.resetEncoders();
            m_rearLeft.resetEncoders();
            m_frontRight.resetEncoders();
            m_rearRight.resetEncoders();
        }
    }

    /** Zeroes the heading of the robot. */
    
    public void zeroHeading() {
        if (ENABLED) {
            m_gyro.reset();
            m_gyro.setAngleAdjustment(180);
            Pose2d pose = getPose();
            Pose2d pose2 = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(0));
            resetOdometry(pose2);
        }
    }
     /*
    //Calibrates the gyro. WHY IS THERE AN error
    public void calculateHeading() {
        if (ENABLED) {
            m_gyro.calibrate();
            while (!m_gyro.isCalibrating()) {
                ;
            }
        }
    }
    */

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Optional<Double> getHeading() {
        return ENABLED ? Optional.of(Rotation2d.fromDegrees(GYRO_ORIENTATION * getGyroAngle()).getDegrees())
                : Optional.empty();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public Optional<Double> getTurnRate() {
        return ENABLED ? Optional.of(m_gyro.getRate() * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0))
                : Optional.empty();
    }

    public Optional<SwerveModule> getFrontLeftModule() {
        return ENABLED ? Optional.of(m_frontLeft) : Optional.empty();
    }

    public Optional<SwerveModule> getFrontRightModule() {
        return ENABLED ? Optional.of(m_frontRight) : Optional.empty();
    }

    public Optional<SwerveModule> getRearLeftModule() {
        return ENABLED ? Optional.of(m_rearLeft) : Optional.empty();
    }

    public Optional<SwerveModule> getRearRightModule() {
        return ENABLED ? Optional.of(m_rearRight) : Optional.empty();
    }

    public Optional<AHRS> getImu() {
        return ENABLED ? Optional.of(m_gyro) : Optional.empty();
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void initialize() {
        if (ENABLED) {
            m_frontLeft = new SwerveModule(
                    Constants.Ports.CAN.FRONT_LEFT_DRIVING,
                    Constants.Ports.CAN.FRONT_LEFT_TURNING,
                    Constants.Ports.ANALOG.FRONT_LEFT_TURNING_ABSOLUTE_ENCODER, false);

            m_frontRight = new SwerveModule(
                    Constants.Ports.CAN.FRONT_RIGHT_DRIVING,
                    Constants.Ports.CAN.FRONT_RIGHT_TURNING,
                    Constants.Ports.ANALOG.FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER, false);

            m_rearLeft = new SwerveModule(
                    Constants.Ports.CAN.REAR_LEFT_DRIVING,
                    Constants.Ports.CAN.REAR_LEFT_TURNING,
                    Constants.Ports.ANALOG.REAR_LEFT_TURNING_ABSOLUTE_ENCODER, false);

            m_rearRight = new SwerveModule(
                    Constants.Ports.CAN.REAR_RIGHT_DRIVING,
                    Constants.Ports.CAN.REAR_RIGHT_TURNING,
                    Constants.Ports.ANALOG.REAR_RIGHT_TURNING_ABSOLUTE_ENCODER, false);

            m_gyro = new AHRS(Port.kMXP);
            // m_gyro.calibrate();
            // while (m_gyro.isCalibrating()) {
            // ;
            // }
            m_gyro.reset();
            m_gyro.zeroYaw();

            m_odometry = new SwerveDriveOdometry(
                    DrivetrainConstants.DRIVE_KINEMATICS,
                    Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getAngle()),
                    new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                    });

            m_frontLeft.calibrateVirtualPosition(FRONT_LEFT_VIRTUAL_OFFSET_RADIANS);
            m_frontRight.calibrateVirtualPosition(FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS);
            m_rearLeft.calibrateVirtualPosition(REAR_LEFT_VIRTUAL_OFFSET_RADIANS);
            m_rearRight.calibrateVirtualPosition(REAR_RIGHT_VIRTUAL_OFFSET_RADIANS);

            resetEncoders();

            //calculateHeading();
            zeroHeading();

            Translation2d initialTranslation = new Translation2d(Units.inchesToMeters(FIELD_LENGTH_INCHES / 2),
                    Units.inchesToMeters(FIELD_WIDTH_INCHES / 2)); // mid field
            Rotation2d initialRotation = Rotation2d.fromDegrees(180);
            m_gyro.setAngleAdjustment(0);
            Pose2d initialPose = new Pose2d(initialTranslation, initialRotation);
            resetOdometry(initialPose);
        }
    }

    public void periodic()
    {
        // outputting offsets
        SmartDashboard.putNumber("Front left angle offset", FRONT_LEFT_VIRTUAL_OFFSET_RADIANS);
        SmartDashboard.putNumber("Front right angle offset", FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS);
        SmartDashboard.putNumber("Rear left angle offset", REAR_LEFT_VIRTUAL_OFFSET_RADIANS);
        SmartDashboard.putNumber("Rear right angle offset", REAR_RIGHT_VIRTUAL_OFFSET_RADIANS);

        // outputting encoder angles for modules

        SmartDashboard.putNumber("Front left angle", m_frontLeft.getTurningAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Front right angle", m_frontRight.getTurningAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Rear left angle", m_rearLeft.getTurningAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Rear right angle", m_rearRight.getTurningAbsoluteEncoder().getPosition());
        
        // angle
        SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
        
        // module encoder velocities
        SmartDashboard.putNumber("Front left velocity", m_frontLeft.getDrivingEncoder().getVelocity());
        SmartDashboard.putNumber("Front right velocity", m_frontRight.getDrivingEncoder().getVelocity());
        SmartDashboard.putNumber("Rear left velocity", m_rearLeft.getDrivingEncoder().getVelocity());
        SmartDashboard.putNumber("Front right velocity", m_rearRight.getDrivingEncoder().getVelocity());
    }


}
