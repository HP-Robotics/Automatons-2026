package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class DriveConstants{
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }
}
