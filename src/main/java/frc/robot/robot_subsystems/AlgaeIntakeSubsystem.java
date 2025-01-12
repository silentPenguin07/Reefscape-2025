package frc.robot.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_leader, m_follower;

    public AlgaeIntakeSubsystem()
    {
        m_leader = new CANSparkMax(Constants.ElevatorConstants.ALGAE_INTAKE_LEADER_MOTOR_ID, MotorType.kBrushless);
        m_follower = new CANSparkMax(Constants.ElevatorConstants.ALGAE_INTAKE_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        m_follower.follow(m_leader, true);
    }

    public void revAlgaeIntake(boolean reverse)
    {
        if (reverse) {m_leader.set(0-0.2);}
        else {m_leader.set(0+0.2);}
    }

    public void brake() {m_leader.set(0);}
}
