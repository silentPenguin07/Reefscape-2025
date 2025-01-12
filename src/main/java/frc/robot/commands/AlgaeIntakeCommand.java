package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeIntakeSubsystem m_subsystem;
    private boolean reverse;

    public AlgaeIntakeCommand(AlgaeIntakeSubsystem m_subsystem, boolean reverse)
    {
        this.m_subsystem = m_subsystem;
        this.reverse = reverse;
        addRequirements(m_subsystem);
    }

    public void initialize() {}

    public void execute() {m_subsystem.revAlgaeIntake(reverse);}

    public void end(boolean interrupted)
    {
        m_subsystem.brake();
    }

    public boolean isFinished() {return false;}
}
