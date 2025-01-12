package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot_subsystems.CoralElevatorSubsystem;
import frc.robot.robot_subsystems.DriveSubsystem;
import frc.robot.robot_subsystems.CoralIntakeSubsystem;

public class RobotContainer {
    
    public static final double GAMEPAD_AXIS_THRESHOLD = 0.2;
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final CoralIntakeSubsystem intakeSubsystem = new CoralIntakeSubsystem();
    private final CoralElevatorSubsystem coralElevatorSubsystem = new CoralElevatorSubsystem();
    
    Joystick driverGamepad = new Joystick(Constants.Ports.CONTROLLER.DRIVER_JOYSTICK);

    // the container for the robot. contains subsystems, OI devices, commands
    public RobotContainer() 
    {
        /*
         * Registering named commands so they can be used to create Event Markers and
         * Autos. All named commands MUST be registered before creating a Pathplanner
         * plan or auto. Any named commands registered after path/auto creation will not
         * be used.
         */
        //NamedCommands.registerCommand("Shooter Rev", new RevShooterCommand(getShooterSubsystem()));
        //NamedCommands.registerCommand("Intake and Shoot", new IntakeShootCommand(getShooterSubsystem(), getCoralIntakeSubsystem()));
        //NamedCommands.registerCommand("Halt", new HaltCommand(getShooterSubsystem(), getCoralIntakeSubsystem()));
        //NamedCommands.registerCommand("Reverse Intake", new RunIntake(intakeSubsystem, true));

        driveSubsystem.initialize();
        

        // configure the trigger bindings
        configureBindings();

    }
    
    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureBindings() {}

    // passes the autonomous command to the main Robot class
    public Command getAutonomousCommand()
    {
        //return autoChooser.getSelected();
        // I'm too lazy to deal with autoChooser :/
        return driveSubsystem.auton();
    }
    
    public DriveSubsystem getDriveSubsystem()
    {
        return driveSubsystem;
    }

    public CoralIntakeSubsystem getCoralIntakeSubsystem()
    {
        return intakeSubsystem;
    }

    public CoralElevatorSubsystem getCoralElevatorSubsystem()
    {
        return coralElevatorSubsystem;
    }

}
