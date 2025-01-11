package entechlib.entech_swerve.motors;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import entechlib.entech_swerve.ConfigConstructionUtil.ControlType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public abstract class SparkController implements SwerveMotor {
    protected final CANSparkBase controller;
    protected final SparkPIDController pidController;
    protected final RelativeEncoder encoder;
    protected ControlType control;
    protected boolean inverted = false;

    protected SparkController(CANSparkBase controller) {
        this.controller = controller;
        this.controller.restoreFactoryDefaults();
        pidController = controller.getPIDController();
        encoder = controller.getEncoder();
        pidController.setOutputRange(-1, 1);
        pidController.setFeedbackDevice(encoder);
        this.controller.setIdleMode(IdleMode.kBrake);
        this.controller.setInverted(inverted);
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setPositionPIDWrappingMinInput(-1);
        pidController.setPositionPIDWrappingMaxInput(1);
    }

    @Override
    public void setControlMethod(ControlType control) {
        this.control = control;
    }

    @Override
    public void setPID(double p, double i, double d, double ff) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    @Override
    public void setCurrentLimit(int limit) {
        controller.setSmartCurrentLimit(limit);
    }

    @Override
    public void completeConfigure() {
        controller.burnFlash();
    }

    @Override
    public void setPositionConversionFactor(double positionConversionFactor) {
        encoder.setPositionConversionFactor(positionConversionFactor);
    }

    @Override
    public void setVelocityConversionFactor(double positionConversionFactor) {
        encoder.setVelocityConversionFactor(positionConversionFactor);
    }

    @Override
    public void set(double speed) {
        controller.set(speed);
    }

    @Override
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setReference(double value) {
        if (control == ControlType.POSITION) {
            pidController.setReference(value, com.revrobotics.CANSparkMax.ControlType.kPosition);
        } else {
            pidController.setReference(value, com.revrobotics.CANSparkMax.ControlType.kVelocity);
        }
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        controller.setInverted(inverted);
    }
}