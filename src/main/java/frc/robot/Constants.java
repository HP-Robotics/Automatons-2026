package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Constants {
    public static class SubsystemConstants {
        public static final boolean useIntake = true;
    }

    public static class IntakeConstants {
        public static final double speed = 1;
    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger intakeTrigger = m_driveJoystick.button(0); // TODO: pick a button number
    }

    public static class MotorIDConstants {
        public static final int intakeMotor = 0;
        // TODO:find this ID
    }
}
