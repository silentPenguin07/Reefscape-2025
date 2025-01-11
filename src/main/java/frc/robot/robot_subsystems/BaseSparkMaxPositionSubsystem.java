package frc.robot.robot_subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.controllers.PositionControllerConfig;

public class BaseSparkMaxPositionSubsystem {
    public enum HomingState {
        FINDING_LIMIT, HOMED, UNINITIALIZED
    }

    public static final int CAN_TIMEOUT_MILLIS = 1000;
    protected CANSparkMax spark;
    private HomingState axisState = HomingState.UNINITIALIZED;
    protected boolean enabled = false;
    private PositionControllerConfig config;
    private RelativeEncoder encoder;
    private SparkLimitSwitch lowerLimit;
    private Optional<Double> requestedPosition = Optional.empty();
    private SparkLimitSwitch upperLimit;
    protected boolean liveMode = false;

    public BaseSparkMaxPositionSubsystem() {

    }

    public void configure(CANSparkMax spark, PositionControllerConfig config) {
        configure(spark,
                config,
                spark.getReverseLimitSwitch(Type.kNormallyOpen),
                spark.getForwardLimitSwitch(Type.kNormallyOpen),
                spark.getEncoder());
    }

    public void configure(CANSparkMax spark, PositionControllerConfig config, SparkLimitSwitch lowerLimit,
            SparkLimitSwitch upperLimit, RelativeEncoder encoder) {
        this.spark = spark;

        if (config.isInverted()) {
            this.lowerLimit = upperLimit;
            this.upperLimit = lowerLimit;
        } else {
            this.lowerLimit = lowerLimit;
            this.upperLimit = upperLimit;
        }

        this.encoder = encoder;
        this.config = config;
        clearRequestedPosition();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void clearRequestedPosition() {
        this.requestedPosition = Optional.empty();
        setMotorSpeedInternal(0.0);
    }

    public double getActualPosition() {
        return getEncoderValue();
    }

    public PositionControllerConfig getConfig() {
        return this.config;
    }

    public double getMotorOutput() {
        return spark.getAppliedOutput();
    }

    public Optional<Double> getRequestedPosition() {
        return requestedPosition;
    }

    public String getStatusString() {
        if (axisState == HomingState.HOMED) {
            if (requestedPosition.isPresent()) {
                if (this.isAtRequestedPosition()) {
                    return "ARRIVED";
                } else {
                    return "MOVING";
                }
            } else {
                return "HOMED";
            }
        } else {
            return this.axisState + "";
        }
    }

    public void home() {
        startHoming();
    }

    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Status:", this::getStatusString, null);
        builder.addDoubleProperty("RequestedPos", this::getRequestedPositionAsDouble, null);
        builder.addDoubleProperty("ActualPos", this::getActualPosition, null);
        builder.addDoubleProperty("MotorOut", this::getMotorOutput, null);
        builder.addBooleanProperty("Homed", this::isHomed, null);
        builder.addBooleanProperty("UpperLimit", this::isAtUpperLimit, null);
        builder.addBooleanProperty("LowerLimit", this::isAtLowerLimit, null);
        if (liveMode) {
            builder.addDoubleProperty("P", this::getP, this::setP);
        }

    }

    /**
     * Live Mode things
     * 
     */
    private void setP(double p) {
        spark.getPIDController().setP(p);
    }

    private double getP() {
        return spark.getPIDController().getP();
    }

    private double getRequestedPositionAsDouble() {
        if (requestedPosition.isPresent()) {
            return requestedPosition.get();
        } else {
            return Constants.INDICATOR_VALUES.POSITION_NOT_SET;
        }
    }

    public boolean isAtLowerLimit() {
        return lowerLimit.isPressed();
    }

    public boolean isAtUpperLimit() {
        return upperLimit.isPressed();
    }

    public boolean isAtRequestedPosition() {
        if (requestedPosition.isPresent()) {
            if (isHomed()) {
                return Math.abs(getActualPosition() - requestedPosition.get()) < config.getPositionTolerance();
            }
        }
        return false;
    }

    public boolean isHomed() {
        ;
        return axisState == HomingState.HOMED;
    }

    public void requestPosition(double requestedPosition) {
        if (isPositionWithinSoftLimits(requestedPosition)) {
            this.requestedPosition = Optional.of(requestedPosition);
            if (axisState == HomingState.UNINITIALIZED) {
                startHoming();
            }
        } else {
            DriverStation.reportWarning("Invalid Position " + requestedPosition, false);
        }
    }

    public void setMotorSpeed(double input) {
        clearRequestedPosition();
        setMotorSpeedInternal(input);
    }

    public void stop() {
        spark.stopMotor();
    }

    public void periodic() {
        switch (axisState) {
            case UNINITIALIZED:
                break;
            case FINDING_LIMIT:
                if (isAtLowerLimit()) {
                    arrivedHome();
                }
                break;
            case HOMED:
                updateRequestedPosition();
        }
    }

    private void setMotorSpeedInternal(double input) {
        spark.set(correctDirection(input));
    }

    private void arrivedHome() {
        setMotorSpeedInternal(0);
        setEncoder(config.getMinPosition());
        axisState = HomingState.HOMED;
    }

    private double correctDirection(double input) {
        if (config.isInverted()) {
            return -input;
        } else {
            return input;
        }
    }

    private double getEncoderValue() {
        return correctDirection(encoder.getPosition());
    }

    private boolean isPositionWithinSoftLimits(double position) {
        return position >= config.getMinPosition() && position <= config.getMaxPosition();
    }

    private void setEncoder(double value) {
        encoder.setPosition(correctDirection(value));
    }

    private void startHoming() {
        // if we are already on the limit switch, we'll get homed in the next update
        // loop
        setMotorSpeedInternal(-config.getHomingSpeedPercent());
        axisState = HomingState.FINDING_LIMIT;
    }

    private void updateRequestedPosition() {
        if (requestedPosition.isPresent()) {
            spark.getPIDController().setReference(correctDirection(requestedPosition.get()),
                    CANSparkMax.ControlType.kPosition);
            spark.getPIDController().setIAccum(0);
        }
    }
}
