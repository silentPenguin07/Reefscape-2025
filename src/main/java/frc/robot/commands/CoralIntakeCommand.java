package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {
    
    private final CoralIntakeSubsystem intakeSubsystem;
    private boolean reverse;

    public CoralIntakeCommand(CoralIntakeSubsystem intakeSubsystem, boolean reverse)
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
