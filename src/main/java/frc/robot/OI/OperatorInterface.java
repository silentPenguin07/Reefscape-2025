package frc.robot.OI;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CommandFactory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CoralElevatorSetPositionCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.XCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInterface {
        private final XboxController driveJoystick = new XboxController(Constants.Ports.CONTROLLER.DRIVER_JOYSTICK);
        private final XboxController operator = new XboxController(Constants.Ports.CONTROLLER.ARM_JOYSTICK);
        public OperatorInterface(CommandFactory commandFactory, RobotContainer robotContainer) {
                /*
                operator controls
                */
                // coral intake/outtake
                new JoystickButton(operator, Constants.Controller.RIGHT_BUMPER).whileTrue(new CoralIntakeCommand(robotContainer.getCoralIntakeSubsystem(), false));
                new JoystickButton(operator, Constants.Controller.LEFT_BUMPER).whileTrue(new CoralIntakeCommand(robotContainer.getCoralIntakeSubsystem(), true));
                // algae intake/outtake (controlled by back trigger axes) + some tolerance
                if (Math.abs(operator.getLeftTriggerAxis()) > 0.05)
                {
                        new AlgaeIntakeCommand(null, false);
                }
                if (Math.abs(operator.getRightTriggerAxis()) > 0.05)
                {
                        new AlgaeIntakeCommand(null, false);
                }
                // ARM TO POSITIONS
                new JoystickButton(operator, Constants.Controller.Y_BUTTON).onTrue(new CoralElevatorSetPositionCommand(Constants.ElevatorConstants.ELEVATOR_ANGLE_HIGH, robotContainer.getCoralElevatorSubsystem()));
                new JoystickButton(operator, Constants.Controller.X_BUTTON).onTrue(new CoralElevatorSetPositionCommand(Constants.ElevatorConstants.ELEVATOR_ANGLE_MID, robotContainer.getCoralElevatorSubsystem()));
                new JoystickButton(operator, Constants.Controller.A_BUTTON).onTrue(new CoralElevatorSetPositionCommand(Constants.ElevatorConstants.ELEVATOR_ANGLE_LOW, robotContainer.getCoralElevatorSubsystem()));

                
                /*
                driver controls
                */
                new JoystickButton(driveJoystick, 2).onTrue(commandFactory.gyroResetCommand());
                new JoystickButton(driveJoystick, 3).onTrue(new XCommand());
                robotContainer.getDriveSubsystem().setDefaultCommand(new DriveCommand(robotContainer.getDriveSubsystem(), driveJoystick));
        }
}