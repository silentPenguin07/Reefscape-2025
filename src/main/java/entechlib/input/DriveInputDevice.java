package entechlib.input;

/**
 * A device that supplies driveInput to be filtered and used in the swerveDrive
 * system.
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public interface DriveInputDevice {
    /**
     * Creates a new driveInput with the current inputs.
     * 
     * 
     * @return current input
     */
    public DriveInput getDriveInput();
}
