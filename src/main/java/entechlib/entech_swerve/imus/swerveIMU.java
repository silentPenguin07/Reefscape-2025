package entechlib.entech_swerve.imus;

public interface swerveIMU {

    double getAngle();

    void setAngleOffset(double offset);

    void zeroYaw();

    void reset();

    double getRate();

}