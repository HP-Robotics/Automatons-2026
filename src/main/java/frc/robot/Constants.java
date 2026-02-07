package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class SubsystemConstants {
        public static final boolean useClimber = true;
    }

    public static class DriveConstants {
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public static class MotorIDConstants {
        public static final int climberMotor = 0; // TODO: find id
    }

    public static class ClimberConstants {
        public static final double climberTopPosition = 0;
        public static final double climberBottomPosition = 0;
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;

    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger climbTrigger = m_driveJoystick.button(0);// TODO: pick a button number
        public static final Trigger climbUpTrigger = m_driveJoystick.button(0); // TODO: pick a button number
        public static final Trigger climbDownTrigger = m_driveJoystick.button(0); // TODO: pick a button number
    }
}
