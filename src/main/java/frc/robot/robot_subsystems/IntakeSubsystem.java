package frc.robot.robot_subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intake;

    public IntakeSubsystem()
    {
        CANSparkMax m_intake = new CANSparkMax(Constants.ArmConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    }

    public void intake(boolean reverse)
    {
        if (!reverse)
        {
            m_intake.set(0.4);
        }
        else{
            m_intake.set(-0.4);
        }
    }

    public void brake()
    {
        m_intake.set(0);
    }

}