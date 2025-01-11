package entechlib.entech_swerve.motors;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkFlexVortex extends SparkController {
    public SparkFlexVortex(int id) {
        super(new CANSparkFlex(id, MotorType.kBrushless));
    }
}
