package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class Constants {
    public static class SubsystemConstants {
        public static final boolean useHopperSubsystem = true;
        public static final boolean useIntake = true;
        public static final boolean useShooter = true;
    }

    public static class IntakeConstants {
        public static final double speed = 1;
    }

    public static class DriveConstants {
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger intakeTrigger = m_driveJoystick.button(0);
        public static final Trigger setShooterTrigger = m_driveJoystick.button(1);
        public static final Trigger stopShooterTrigger = m_driveJoystick.button(2);
        public static final Trigger magicShooterTrigger = m_driveJoystick.button(3);
        public static final Trigger adjustableShooterTrigger = m_driveJoystick.button(4);
        // TODO: pick a button number for all of these
        public static final Trigger hopperTrigger = m_driveJoystick.button(1); // TODO: pick a button number
    }

    public static class MotorIDConstants {
        public static final int intakeMotor = 0;
        // TODO:find this ID
        public static final int shooterMotor1 = 53;
        public static final int shooterMotor2 = 54;
        public static final int HopperMotorSpinner = 0;
        public static final int HopperMotorOutake = 42; // invert this
        // TODO: set motor ID
    }

    public static class ShooterConstants {
        public static final double shootingSpeed = 1;
        // TODO:find a value for a fixed speed mode
        public static final double idleSpeed = 0.1;
        // TODO: pick this
    }

    public static class LimelightConstants {
        public static final double inToM = 0.0254;
        public static final AprilTagFieldLayout field = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Pose2d aprilTagList[] = LimelightSubsystem.getFieldTags(field);
        public static final int disabledThrottle = 200;
        public static final double imuAssist = 0.005;
    }

    public static class HopperConstants {
        public static final double spinnerSpeed = .5;
        public static final double outakeSpeed = .8;
        // TODO: hopper/outake motors must turn counter clock-wise.
    }
}
