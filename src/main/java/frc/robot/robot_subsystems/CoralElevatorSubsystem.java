package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class CoralElevatorSubsystem extends SubsystemBase {
    
    // Motor controllers
    private CANSparkMax m_elevator_leader; // NEO for moving arm upwards
    private CANSparkMax m_elevator_follower; // follower NEO
    private double gravityControl; // TODO: check this calculation

    // limiters TODO: test these
    public double arm_max = Constants.ElevatorConstants.ELEVATOR_ARM_MAX;
    public double arm_min = Constants.ElevatorConstants.ELEVATOR_ARM_MIN;

    public CoralElevatorSubsystem()
    {
        m_elevator_leader = new CANSparkMax(Constants.ElevatorConstants.ELEVATOR_LEAD_MOTOR_ID, MotorType.kBrushless);
        m_elevator_follower = new CANSparkMax(Constants.ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        
        // follower turns in opposite direction because it's on the other end of shaft
        m_elevator_follower.follow(m_elevator_leader, true);
    }

    public double getElevatorPosition()
    {
        /*
            .getEncoder() -> relative
            .getAbsoluteEncoder() -> absolute
            REMEMBER: absolute encoder works with alternate adapter, but remember to convert from rotations
        */
        return m_elevator_leader.getEncoder().getPosition(); // maybe just getEncoder()???
    }

    public void setElevatorSpeed(double speed)
    {
        m_elevator_leader.set(speed);
    }

    public void stopElevator()
    {
        m_elevator_leader.set(0);
    }

    public double getGravityControl()
    {
        return gravityControl;
    }

    public void periodic()
    {
        gravityControl = Math.sin((getElevatorPosition() / 70 * 2 * Math.PI) + Math.PI / 2) * 
            Constants.ElevatorConstants.ELEVATOR_GRAVITY_CONST;

        SmartDashboard.putNumber("Elevator Angle", getElevatorPosition());
        SmartDashboard.putNumber("Gravity Control", gravityControl);
    }

}
