package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Constants {
    public static class SubsystemConstants {
        public static final boolean useIntake = true;
    }

    public static class IntakeConstants {
        public static final double speed = 0.3;
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;

        // set slot 0 gains
        public static final double leftkS = 0; // Add 0.25 V output to overcome static friction
        public static final double leftkV = 0; // A velocity target of 1 rps results in 0.12 V output
        public static final double leftkA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double leftkP = 2; // A position error of 2.5 rotations results in 12 V output
        public static final double leftkI = 0; // no output for integrated error
        public static final double leftkD = 0; // A velocity error of 1 rps results in 0.1 V output
        // set Motion Magic settings
        public static final double leftMotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
        public static final double leftMotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
        public static final double leftMotionMagicJerk = 1; // Target jerk of 1600 rps/s/s (0.1 seconds)
        public static final double leftExtendPosition = 3.08;
        public static final double leftRetractPosition = 0;

        // set slot 0 gains
        public static final double rightkS = 0; // Add 0.25 V output to overcome static friction
        public static final double rightkV = 0; // A velocity target of 1 rps results in 0.12 V output
        public static final double rightkA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double rightkP = 2; // A position error of 2.5 rotations results in 12 V output
        public static final double rightkI = 0; // no output for integrated error
        public static final double rightkD = 0; // A velocity error of 1 rps results in 0.1 V output
        // set Motion Magic settings
        public static final double rightMotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
        public static final double rightMotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
        public static final double rightMotionMagicJerk = 1; // Target jerk of 1600 rps/s/s (0.1 seconds)
        public static final double rightExtendPosition = -3.08;
        public static final double rightRetractPosition = 0;
    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger intakeTrigger = m_driveJoystick.button(0); // TODO: pick a button number
        public static final Trigger intakeExtendTrigger = m_driveJoystick.button(2);
    }

    public static class MotorIDConstants {
        public static final int intakeSpinMotor = 33;
        public static final int intakeExtendRightMotor = 31;
        public static final int intakeExtendLeftMotor = 32;
    }
}
