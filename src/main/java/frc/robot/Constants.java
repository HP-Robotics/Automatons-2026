package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class Constants {
    public static class SubsystemConstants {
        public static final boolean useClimber = false;
        public static final boolean useHood = true;
    }

    public static class DriveConstants {
        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public static class LimelightConstants {
        public static final double inToM = 0.0254;
        public static final AprilTagFieldLayout field = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Pose2d aprilTagList[] = LimelightSubsystem.getFieldTags(field);
        public static final int disabledThrottle = 200;
        public static final double imuAssist = 0.005;
    }

    public static class MotorIDConstants {
        public static final int climberMotor = 0; // TODO: find id
        public static final int hoodMotor = 52;
    }

    public static class ClimberConstants {
        public static final double climberTopPosition = 0;
        public static final double climberBottomPosition = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0;

    }

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger climbTrigger = m_driveJoystick.button(0);// TODO: pick a button number
        public static final Trigger climbUpTrigger = m_driveJoystick.button(0); // TODO: pick a button number
        public static final Trigger climbDownTrigger = m_driveJoystick.button(0); // TODO: pick a button number
    }

    public static class HoodConstants {
        public static final double hoodBottom = -0.41;
        public static final double hoodTop = 2.15;
        public static final double kP = 1.5;
        public static final double kD = 0.01;
        public static final double maxSpeed = 0.2;
        // hood speed 0.04 was safe.
    }
}
