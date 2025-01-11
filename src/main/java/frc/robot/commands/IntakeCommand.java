package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private boolean reverse;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean reverse)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.reverse = reverse;
    }

    public void initialize()
    {
        intakeSubsystem.brake();
    }

    public void execute()
    {
        intakeSubsystem.intake(reverse);
    }

    public void end(boolean interrupted)
    {
        intakeSubsystem.brake();
    }

    public boolean isFinished()
    {
        return false;
    }

}
