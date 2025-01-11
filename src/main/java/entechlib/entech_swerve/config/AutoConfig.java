package entechlib.entech_swerve.config;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class AutoConfig {
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController rotController;

    public PIDController getXController() {
        return this.xController;
    }

    public void setXController(PIDController xController) {
        this.xController = xController;
    }

    public PIDController getYController() {
        return this.yController;
    }

    public void setYController(PIDController yController) {
        this.yController = yController;
    }

    public ProfiledPIDController getRotController() {
        return this.rotController;
    }

    public void setRotController(ProfiledPIDController rotController) {
        this.rotController = rotController;
    }
}
