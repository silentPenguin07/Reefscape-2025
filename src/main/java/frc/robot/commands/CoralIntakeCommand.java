package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot_subsystems.CoralIntakeSubsystem;

public class CoralIntakeCommand extends Command {
    
    private final CoralIntakeSubsystem coralIntakeSubsystem;
    private boolean reverse;

    public CoralIntakeCommand(CoralIntakeSubsystem coralIntakeSubsystem, boolean reverse)
    {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.reverse = reverse;
        addRequirements(coralIntakeSubsystem);
    }

    public void initialize()
    {
        coralIntakeSubsystem.brake();
    }

    public void execute()
    {
        coralIntakeSubsystem.intake(reverse);
    }

    public void end(boolean interrupted)
    {
        coralIntakeSubsystem.brake();
    }

    public boolean isFinished()
    {
        return false;
    }

}
