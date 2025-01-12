package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.robot_subsystems.CoralElevatorSubsystem;

public class CoralElevatorSetPositionCommand extends Command {
    
    private CoralElevatorSubsystem m_subsystem;
    private double position;
    private double error;
    private double kP = Constants.ElevatorConstants.kP;

    public CoralElevatorSetPositionCommand(double position, CoralElevatorSubsystem m_Subsystem)
    {
        this.position = position;
        this.m_subsystem = m_Subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize()
    {
        // intentionally blank
    }

    @Override
    public void execute()
    {
        this.error = position - m_subsystem.getElevatorPosition();
        double output = kP * error;

        if (Math.abs(output) > Constants.ElevatorConstants.ELEVATOR_MAX_SPEED) // cap the goddamn power
        {
            output = Math.copySign(Constants.ElevatorConstants.ELEVATOR_MAX_SPEED, output);
        }

        // TODO: not assigning a min power just yet

        m_subsystem.setElevatorSpeed(output - m_subsystem.getGravityControl()); // TODO: Gravity control???

    }

    @Override
    public void end(boolean interrupted)
    {
        m_subsystem.stopElevator();
    }

    @Override
    public boolean isFinished() // stop if close enough
    {
        return Math.abs(error) <= 0.5;
    }

}
