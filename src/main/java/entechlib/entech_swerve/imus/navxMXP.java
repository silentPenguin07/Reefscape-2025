package entechlib.entech_swerve.imus;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class navxMXP extends navxGyro {
    public navxMXP() {
        super(new AHRS(Port.kMXP));
    }
}
