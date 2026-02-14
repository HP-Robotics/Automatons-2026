// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {
    private double m_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                          // speed
    private double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                        // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry m_logger = new Telemetry(m_maxSpeed);

    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();

    public final CommandSwerveDrivetrain m_drivetrain = (SubsystemConstants.useDrive)
            ? TunerConstants.createDrivetrain()
            : null;
    public final IntakeSubsystem m_intakeSubsystem = (SubsystemConstants.useIntake) ? new IntakeSubsystem() : null;
    public final ShooterSubsystem m_shooterSubsystem = (SubsystemConstants.useShooter) ? new ShooterSubsystem() : null;
    public final TurretSubsystem m_turretSubsystem = (SubsystemConstants.useTurret) ? new TurretSubsystem() : null;
    public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    private final CommandXboxController m_joystick = new CommandXboxController(0); // TODO: Use controller constants
    public final ClimberSubsystem m_climberSubsystem = (SubsystemConstants.useClimber) ? new ClimberSubsystem() : null;
    public final HoodSubsystem m_hoodSubsystem = SubsystemConstants.useHood ? new HoodSubsystem() : null;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        if (SubsystemConstants.useDrive) {
            m_drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    m_drivetrain.applyRequest(
                            () -> m_drivetrain.applySetpointGenerator(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    MathUtil.applyDeadband(m_joystick.getLeftY(), 0.1) * m_maxSpeed,
                                    MathUtil.applyDeadband(m_joystick.getLeftX(), 0.1) * m_maxSpeed,
                                    MathUtil.applyDeadband(-m_joystick.getRightX(), 0.1) * m_maxAngularRate,
                                    m_drivetrain.getRotation3d().toRotation2d()))));
        }
        if (SubsystemConstants.useIntake) {
            ControllerConstants.intakeTrigger.whileTrue(m_intakeSubsystem.Intake());
        }

        if (SubsystemConstants.useShooter) {
            // ControllerConstants.setShooterTrigger.whileTrue(m_shooterSubsystem.fixedShooter());
            // ControllerConstants.stopShooterTrigger.whileTrue(m_shooterSubsystem.stopShooter());
            ControllerConstants.adjustableShooterTrigger.whileTrue(m_shooterSubsystem.AdjustableShooter());
            // ControllerConstants.magicShooterTrigger.whileTrue(m_shooterSubsystem.magicShooter());
        }

        if (SubsystemConstants.useHopper) {
            ControllerConstants.hopperTrigger.whileTrue(m_hopperSubsystem.RunHopper());

        }

        if (SubsystemConstants.useHopper && SubsystemConstants.useShooter) {

            ControllerConstants.shootTrigger.whileTrue(
                    new ParallelCommandGroup(m_shooterSubsystem.AdjustableShooter(),
                            new SequentialCommandGroup(new WaitUntilCommand(m_shooterSubsystem::atSpeed),
                                    m_hopperSubsystem.RunHopper())));
            // First, spin up the shooter
            // Wait until shoot is spinning
            // Run hopper
        }
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        if (SubsystemConstants.useDrive) {
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                    m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

            // These are neat but we probably don't need them
            // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
            m_joystick.b().whileTrue(m_drivetrain.applyRequest(
                    () -> point.withModuleDirection(new Rotation2d(-m_joystick.getLeftY(), -m_joystick.getLeftX()))));

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            m_joystick.back().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
            m_joystick.back().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
            m_joystick.start().and(m_joystick.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
            m_joystick.start().and(m_joystick.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));
            // Reset the field-centric heading on left bumper press.
            m_joystick.button(8).onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
            m_drivetrain.registerTelemetry(m_logger::telemeterize);
        }

        if (SubsystemConstants.useClimber) {
            ControllerConstants.climbTrigger.whileTrue(m_climberSubsystem.Climb());
            ControllerConstants.climbUpTrigger.whileTrue(m_climberSubsystem.Climb());
            ControllerConstants.climbDownTrigger.whileTrue(m_climberSubsystem.Climb());
        }

        if (SubsystemConstants.useHood) {
            m_joystick.button(1).whileTrue(m_hoodSubsystem.hoodFromNetworkTables());
        }

        if (SubsystemConstants.useTurret) {
            ControllerConstants.runTurretTrigger.whileTrue(new StartEndCommand(m_turretSubsystem::runTurret,
                    m_turretSubsystem::stopTurret, m_turretSubsystem));
            ControllerConstants.calibrateTurretTrigger.whileTrue(m_turretSubsystem.Calibrate());
            ControllerConstants.turnTurretToTargetTrigger.whileTrue(m_turretSubsystem.RotateTurret());
        }
    }

    public void staticShot() {
        Pose2d pose = m_drivetrain.getState().Pose;
        double distance = pose.getTranslation().getDistance(FieldConstants.hub);
        // Get the hub as a Pose2d (vector representing where it is on the field) -->
        // find the hub in robot relative coordinates --> get the angle with the x axis
        // (robot-facing direction)
        // TODO: factor in turret position compared to robot middle
        Rotation2d angleToHub = new Pose2d(FieldConstants.hub, new Rotation2d()).relativeTo(pose).getTranslation()
                .getAngle();

        Matrix<N2, N1> staticShot = ShooterConstants.distanceToStaticShot.get(distance);
        // turret.pointin(angleToHub); TODO: merge branch to use the turret function
        // hood.setAngle(staticShot.get(1,0));
        // m_shooterSubsystem.setSpeed(staticShot.get(0, 0));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                m_drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                m_drivetrain.applyRequest(() -> idle));
    }

    public void periodic() {
        for (VisionMeasurement visionMeasurement : m_limelightSubsystem.getAllLimelightData()) {
            m_drivetrain.addVisionMeasurement(visionMeasurement.m_visionPose, visionMeasurement.m_timeStamp);
        }
    }
}
