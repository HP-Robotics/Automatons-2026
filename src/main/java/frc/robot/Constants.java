package frc.robot;


import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;

public final class Constants {
    public static class SubsystemConstants {
        public static final boolean useDrive = false;
        public static final boolean useIntake = false;
        public static final boolean useShooter = false;
        public static final boolean useTurret = true;
    }

    public static class PortConstants {
        public static final int turretLimitPort = 9;
        public static final int ledPort = 0; // TODO: get real values
    }

    public static class IntakeConstants {
        public static final double speed = 1;
    }
    // real actual change
  
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

    public static class ControllerConstants {
        public static final CommandJoystick m_driveJoystick = new CommandJoystick(0);
        public static final Trigger intakeTrigger = m_driveJoystick.button(0);
        public static final Trigger setShooterTrigger = m_driveJoystick.button(1);
        public static final Trigger stopShooterTrigger = m_driveJoystick.button(2);
        public static final Trigger magicShooterTrigger = m_driveJoystick.button(3);
        public static final Trigger adjustableShooterTrigger = m_driveJoystick.button(4);
        public static final Trigger runTurretTrigger = m_driveJoystick.button(5);
        public static final Trigger calibrateTurretTrigger = m_driveJoystick.button(6);
        public static final Trigger turnTurretToTargetTrigger = m_driveJoystick.button(7);
        // TODO: pick a button number for all of these

    }

    public static class MotorIDConstants {
        public static final int intakeMotor = 0;
        // TODO:find this ID
        public static final int shooterMotor1 = 1;
        public static final int shooterMotor2 = 2;
        // TODO: actually find the IDs of the shooter motors
        public static final int turretMotor = 10;
    }

    public static class ShooterConstants {
        public static final double shootingSpeed = 1;
        // TODO:find a value for a fixed speed mode
        public static final double idleSpeed = 0.1;
        // TODO: pick this

        public static final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToStaticShot = fillTreeMap();

        public static InterpolatingMatrixTreeMap<Double, N2, N1> fillTreeMap() {
            // TreeMap.put(double) use this function to add to tree map
            InterpolatingMatrixTreeMap<Double, N2, N1> output = new InterpolatingMatrixTreeMap<Double, N2, N1>();
            // output.put(2.0, MatBuilder.fill(Nat.N2(), Nat.N1(), 1.0, 2.0));
            return output;
        }

    }

    public static class TurretConstants {
        public static final double turretSpeed = 0.042; // safe speed for now
        public static final double calibrationPosition = 0.0; // TODO: find the real position, maybe limelight?
        public static final double limitSwitchDegrees = 90.0; // TODO: find real value
        public static final double bottomLimitPosition = -200.0; // TODO: find real value
        public static final double topLimitPosition = 200.0;
        public static final double errorTolerance = 0.0; // find real value
        public static final double encoderCPR = 1.0; // TODO: clarify value
        public static final double gearRatio = 10.4167;
        public static final double distanceToLimitThreshold = 5.0; // in robot relative degrees
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double maxForwardDutyCycle = 0.042;
        public static final double maxReverseDutyCycle = -0.042;
    }

    public static class FieldConstants {
        public static final Translation2d hub = new Translation2d(1.0, 0.0); // TODO: get real coordinates
    }

    public static class LEDConstants {
        public static final LEDPattern defaultPattern = LEDPattern.solid(Color.kSandyBrown);
    }


}
