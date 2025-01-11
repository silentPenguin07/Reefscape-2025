package entechlib.input;

public class DriveInput {
    private double x;
    private double y;
    private double rot;

    public DriveInput(double xSpeed, double ySpeed, double rotationSpeed) {
        x = xSpeed;
        y = ySpeed;
        rot = rotationSpeed;
    }

    public double getXSpeed() {
        return this.x;
    }

    public double getYSpeed() {
        return this.y;
    }

    public double getRotationSpeed() {
        return this.rot;
    }
}
