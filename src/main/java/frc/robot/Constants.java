package frc.robot;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.LimelightSubsystem;

public final class Constants {
    public static class SubsystemConstants {
        public static final boolean useDrive = true;
        public static final boolean useTurret = true;
        public static final boolean useHopper = true;
        public static final boolean useIntake = true;
        public static final boolean useShooter = true;
        public static final boolean useClimber = true;
        public static final boolean useHood = true;
    }

    public static class PortConstants {
        public static final int turretLimitPort = 11;
    }

    public static class IntakeConstants {
        public static final double speed = 0.5;
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

    public static class DriveConstants {
        public enum SelectedSysIdRoutine {
            NONE,
            TRANSLATE,
            ROTATE,
            STEER
        }

        public static final double maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double moduleMaxRotationPerSecond = 4.8;
        public static final SelectedSysIdRoutine sysIdRoutine = SelectedSysIdRoutine.STEER;
    }

    public static class ControllerConstants {
        public static final CommandXboxController m_driveJoystick = new CommandXboxController(0);
        public static final CommandXboxController m_opJoystick = new CommandXboxController(1);

        public static final DoubleSupplier m_leftAxisY = () -> m_driveJoystick.getRawAxis(1);
        public static final DoubleSupplier m_leftAxisX = () -> m_driveJoystick.getRawAxis(0);
        public static final DoubleSupplier m_rightAxisX = () -> m_driveJoystick.getRawAxis(4);

        public static final Trigger intakeTrigger = new Trigger(m_driveJoystick.getHID()::getBButton);
        // public static final Trigger stopShooterTrigger = new
        // Trigger(m_driveJoystick.getHID()::getBButton);
        public static final Trigger magicShooterTrigger = new Trigger(m_driveJoystick.getHID()::getXButton);
        public static final Trigger ShooterNetworkTablesModeTrigger = new Trigger(m_driveJoystick.getHID()::getYButton);
        // public static final Trigger runTurretTrigger = new
        // Trigger(m_driveJoystick.getHID()::getLeftBumperButton);
        public static final Trigger calibrateTurretTrigger = new Trigger(
                m_driveJoystick.getHID()::getRightBumperButton);
        public static final Trigger setFieldCentricTrigger = new Trigger(m_driveJoystick.getHID()::getBackButton);
        public static final Trigger manualHopperTrigger = new Trigger(m_opJoystick.getHID()::getAButton);
        public static final Trigger climbUpTrigger = m_opJoystick.povUp();
        public static final Trigger climbDownTrigger = m_opJoystick.povDown();
        public static final Trigger calibrateClimberTrigger = new Trigger(m_opJoystick.getHID()::getXButton);
        public static final Trigger runStaticShotTrigger = new Trigger(m_driveJoystick.getHID()::getLeftBumperButton);

        // TODO: merge the sysID routine selection and make these do something
        public static final Trigger sysIdDynamicForward = new Trigger(m_driveJoystick.getHID()::getBackButton)
                .and(m_driveJoystick.getHID()::getYButton);
        public static final Trigger sysIdDynamicReverse = new Trigger(m_driveJoystick.getHID()::getBackButton)
                .and(m_driveJoystick.getHID()::getXButton);
        public static final Trigger sysIdQuasistaticForward = new Trigger(m_driveJoystick.getHID()::getStartButton)
                .and(m_driveJoystick.getHID()::getYButton);
        public static final Trigger sysIdQuasistaticReverse = new Trigger(m_driveJoystick.getHID()::getStartButton)
                .and(m_driveJoystick.getHID()::getXButton);
    }

    public static class MotorIDConstants {
        public static final int shooterMotor1 = 53;
        public static final int shooterMotor2 = 54;
        public static final int HopperMotorSpinner = 43;
        public static final int HopperMotorUplifter = 42;
        public static final int climberMotor = 60;
        public static final int hoodMotor = 52;
        public static final int turretMotor = 21; // TODO: fix
        public static final int intakeSpinMotor = 33;
        public static final int intakeExtendRightMotor = 31;
        public static final int intakeExtendLeftMotor = 32;
    }

    public static class ShooterConstants {
        public static final double defaultShootingSpeed = 75; // rotations per second
        public static final double idleSpeed = 10; // rotations per second
        // TODO: pick this
        public static final double kP = 10;
        public static final double kS = 5.8;
        public static final double kV = 0.009;

        public static final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToStaticShot = fillTreeMap();

        public static InterpolatingMatrixTreeMap<Double, N2, N1> fillTreeMap() {
            // TreeMap.put(double) use this function to add to tree map
            InterpolatingMatrixTreeMap<Double, N2, N1> output = new InterpolatingMatrixTreeMap<Double, N2, N1>();
            output.put(1.36, MatBuilder.fill(Nat.N2(), Nat.N1(), 50, 1.15));
            output.put(2.54, MatBuilder.fill(Nat.N2(), Nat.N1(), 60, 2.05));
            output.put(3.68, MatBuilder.fill(Nat.N2(), Nat.N1(), 62, 2.5));
            output.put(4.36, MatBuilder.fill(Nat.N2(), Nat.N1(), 68, 2.5));
            output.put(5.49, MatBuilder.fill(Nat.N2(), Nat.N1(), 74, 2.5));
            return output;
        }

        public static final double lookAheadTime = 0.1;

        public static final double shooterErrorThreshold = 1;

    }

    public static class LimelightConstants {
        public static final double inToM = 0.0254;
        public static final AprilTagFieldLayout field = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Pose2d aprilTagList[] = LimelightSubsystem.getFieldTags(field);
        public static final int disabledThrottle = 200;
        public static final double imuAssist = 0.005;
    }

    public static class TurretConstants {
        public static final double turretSpeed = 0.042; // safe speed for now
        public static final double calibrationPosition = 0.0;
        public static final double limitSwitchDegrees = 171.0;
        public static final double bottomLimitPosition = 93.0;
        public static final double topLimitPosition = 363.0;
        public static final double errorTolerance = 0.0; // TODO: find real value
        public static final double encoderCPR = 1.0;
        public static final double gearRatio = 10.4167;
        public static final double distanceToLimitThreshold = 5.0; // in robot relative degrees
        public static final double kP = 0.4;
        public static final double kI = 0;
        public static final double kD = 0.025;
        public static final double maxForwardDutyCycle = 0.5;
        public static final double maxReverseDutyCycle = -0.5;
        public static final Translation2d centerPosition = new Translation2d(-6.5 * .0254, 6.5 * .0254);
    }

    public static class FieldConstants {
        public static final Translation2d blueHub = new Translation2d(4.625594, 4.034536);
        public static final Translation2d redHub = FlippingUtil.flipFieldPosition(blueHub);
        public static final Translation2d blueDepotSide = new Translation2d(2.54, 6.02);
        public static final Translation2d blueOutpostSide = new Translation2d(2.54, 2.01);
        public static final double blueAllianceZoneX = 4.61;
        public static final double redAllianceZoneX = 11.91;
        public static final double centerLineY = 4.02;
        public static final Translation2d redDepotSide = FlippingUtil.flipFieldPosition(blueDepotSide);
        public static final Translation2d redOutpostSide = FlippingUtil.flipFieldPosition(blueOutpostSide);
    }

    public static class HopperConstants {
        public static final double spinnerSpeed = .5;
        public static final double uplifterSpeed = 80; // rotations per second

        public static final double kP = 10;
        public static final double kS = 3.4;
        public static final double kV = 0.006;
    }

    public static class ClimberConstants {
        public static final double climberTopPosition = 28;
        public static final double climberBottomPosition = 1;
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0;
        public static final double climberCalibrateSpeed = -0.15;

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
