package entechlib.entech_swerve.imus;

import com.kauailabs.navx.frc.AHRS;

public abstract class navxGyro implements swerveIMU {
    private final AHRS navx;

    protected navxGyro(AHRS navx) {
        this.navx = navx;
    }

    @Override
    public double getAngle() {
        return navx.getAngle();
    }

    @Override
    public void setAngleOffset(double offset) {
        navx.setAngleAdjustment(offset);
    }

    @Override
    public void zeroYaw() {
        navx.zeroYaw();
    }

    @Override
    public void reset() {
        navx.reset();
    }

    @Override
    public double getRate() {
        return navx.getRate();
    }
}
