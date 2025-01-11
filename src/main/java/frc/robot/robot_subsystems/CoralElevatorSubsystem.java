package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class CoralElevatorSubsystem extends SubsystemBase {
    
    // Motor controllers
    private CANSparkMax m_elevator_arm; // NEO for moving arm upwards

    // limiters TODO: test these
    public double arm_max = Constants.ArmConstants.ELEVATOR_ARM_MAX;
    public double arm_min = Constants.ArmConstants.ELEVATOR_ARM_MIN;

    public CoralElevatorSubsystem()
    {
        m_elevator_arm = new CANSparkMax(Constants.ArmConstants.ELEVATOR_LEAD_MOTOR_ID, MotorType.kBrushless);
    }

    public double getArmPosition()
    {
        return m_elevator_arm.getAbsoluteEncoder().getPosition(); // maybe just getEncoder()???
    }

    public void setElevatorSpeed(double speed)
    {
        m_elevator_arm.set(speed);
    }

    public void stopElevator()
    {
        m_elevator_arm.set(0);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Angle", getArmPosition());
    }

}
