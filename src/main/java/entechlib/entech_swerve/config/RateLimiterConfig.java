package entechlib.entech_swerve.config;

/**
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public class RateLimiterConfig {
    /** radians per second */
    private double directionalSlewRate = 1.2;
    /** percent per second (1 = 100%) */
    private double magnitudeSlewRate = 1.8;
    /** percent per second (1 = 100%) */
    private double rotationalSlewRate = 2.0;

    public RateLimiterConfig() {
    }

    public RateLimiterConfig(double directionalSlewRate, double magnitudeSlewRate, double rotationalSlewRate) {
        this.directionalSlewRate = directionalSlewRate;
        this.magnitudeSlewRate = magnitudeSlewRate;
        this.rotationalSlewRate = rotationalSlewRate;
    }

    public double getDirectionalSlewRate() {
        return this.directionalSlewRate;
    }

    public void setDirectionalSlewRate(double directionalSlewRate) {
        this.directionalSlewRate = directionalSlewRate;
    }

    public double getMagnitudeSlewRate() {
        return this.magnitudeSlewRate;
    }

    public void setMagnitudeSlewRate(double magnitudeSlewRate) {
        this.magnitudeSlewRate = magnitudeSlewRate;
    }

    public double getRotationalSlewRate() {
        return this.rotationalSlewRate;
    }

    public void setRotationalSlewRate(double rotationalSlewRate) {
        this.rotationalSlewRate = rotationalSlewRate;
    }
}
