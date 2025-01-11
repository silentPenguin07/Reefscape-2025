package entechlib.entech_swerve.motors;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

/**
 * A swerve motor for a swerve module, either turning or driving.
 * 
 * 
 * @author <a href="https://github.com/WhyDoesGodDoThis">Andrew Heitkamp</a>
 */
public class SparkMaxNeo extends SparkController {
    public SparkMaxNeo(int id) {
        super(new CANSparkMax(id, MotorType.kBrushless));
    }
}
