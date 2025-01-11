package frc.robot.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandFactory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralElevatorSetPositionCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.XCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInterface {

        private final XboxController driveJoystick = new XboxController(Constants.Ports.CONTROLLER.DRIVER_JOYSTICK);
        private final XboxController operator = new XboxController(Constants.Ports.CONTROLLER.ARM_JOYSTICK);

        public OperatorInterface(CommandFactory commandFactory, RobotContainer robotContainer) {
                
                /*
                operator controls
                */
                new JoystickButton(operator, Constants.Controller.X_BUTTON).whileTrue(new IntakeCommand(robotContainer.getIntakeSubsystem(), false)); // run intake
                
                // ARM TO HIGH POSITION
                new JoystickButton(operator, Constants.Controller.Y_BUTTON).onTrue(new CoralElevatorSetPositionCommand(50, robotContainer.getCoralElevatorSubsystem()));

                // driver controls
                new JoystickButton(driveJoystick, 2).onTrue(commandFactory.gyroResetCommand());
                new JoystickButton(driveJoystick, 3).onTrue(new XCommand());
                robotContainer.getDriveSubsystem()
                                .setDefaultCommand(new DriveCommand(robotContainer.getDriveSubsystem(), driveJoystick));
        }
}