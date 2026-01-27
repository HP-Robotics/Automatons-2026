package frc.robot;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class Constants {
     public static class SubsystemConstants {
        public static final boolean useClimber = true;
    }
    public static class DriveConstants{
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public static class MotorIDConstants{
        public static final int climberMotor = 0; //TODO: find id 
    }

    public static class climberConstants{

    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger climberUpTrigger = m_driveJoystick.button(0); // TODO: pick a button number
        public static final Trigger ClimberDownTrigger = m_driveJoystick.button(0); // TODO: pick a button number
    }
}
