package frc.robot;
import entechlib.commands.EntechCommand;
import frc.robot.commands.GyroReset;
import frc.robot.robot_subsystems.DriveSubsystem;

public class CommandFactory{
    private DriveSubsystem driveSubsystem;

    public CommandFactory(RobotContainer robotContiner)
    {
        this.driveSubsystem = robotContiner.getDriveSubsystem();
    }

    public EntechCommand gyroResetCommand() {
        return new GyroReset(driveSubsystem);
    }
}