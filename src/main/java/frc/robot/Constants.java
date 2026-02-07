package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class Constants {
    public static class SubsystemConstants {
        public static final boolean useHopperSubsystem = true;
    }

    public static class DriveConstants {
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger hopperTrigger = m_driveJoystick.button(1); // TODO: pick a button number
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
        public static final double outakeSpeed = .5;
        // TODO: hopper/outake motors must turn counter clock-wise.
    }

    public static class MotorIDConstants {
        public static final int HopperMotorSpinner = 0;
        public static final int HopperMotorOutake = 0;
        // TODO: set motor ID
    }
}
