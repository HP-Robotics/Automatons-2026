package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Constants {
    public static class SubsystemConstants {
        public static final boolean useIntake = true;
        public static final boolean useShooter = true;
    }

    public static class IntakeConstants {
        public static final double speed = 1;
    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger intakeTrigger = m_driveJoystick.button(0);
        public static final Trigger setShooterTrigger = m_driveJoystick.button(1);
        public static final Trigger stopShooterTrigger = m_driveJoystick.button(2);
        public static final Trigger magicShooterTrigger = m_driveJoystick.button(3);
        public static final Trigger adjustableShooterTrigger = m_driveJoystick.button(4);
        // TODO: pick a button number for all of these

    }

    public static class MotorIDConstants {
        public static final int intakeMotor = 0;
        // TODO:find this ID
        public static final int shooterMotor1 = 1;
        public static final int shooterMotor2 = 2;
        // TODO: actually find the IDs of the shooter motors
    }

    public static class ShooterConstants {
        public static final double shootingSpeed = 1;
        // TODO:find a value for a fixed speed mode
        public static final double idleSpeed = 0.1;
        // TODO: pick this

    }

}
