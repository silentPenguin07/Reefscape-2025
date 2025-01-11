package frc.robot.controllers;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class CANMotorController implements MotorController{
    public static enum MotorControllerType {
        SMAX_BRUSHED,
        SMAX_BRUSHLESS
    }

    private final MotorControllerType type;
    private CANSparkMax motorControllerSMAX;
    
    // constructor initializes the motor controller
    public CANMotorController(int CANID, MotorControllerType Type) {
        type = Type;

        switch (type) {
            case SMAX_BRUSHED:
                motorControllerSMAX = new CANSparkMax(CANID, MotorType.kBrushed);
            case SMAX_BRUSHLESS:
                motorControllerSMAX = new CANSparkMax(CANID, MotorType.kBrushless);
        }
    }

    @Override
    public void set(double speed) {
        motorControllerSMAX.set(speed);
    }

    @Override
    public double get() {
        return motorControllerSMAX.get();
    }

    @Override
    public void disable() {
        motorControllerSMAX.disable();
    }

    @Override
    public void stopMotor() {
        motorControllerSMAX.stopMotor();
    }
    
    @Override
    public void setInverted(boolean inverted) {
        motorControllerSMAX.setInverted(inverted);
    }

    @Override
    public boolean getInverted() {
        return motorControllerSMAX.getInverted();
    }

    public void feed() {
        motorControllerSMAX.set(get());
    }
}
