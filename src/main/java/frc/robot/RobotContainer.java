// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry m_logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                m_drivetrain.applySetpointGenerator(ChassisSpeeds.fromFieldRelativeSpeeds(
                    MathUtil.applyDeadband(-m_joystick.getLeftY(), 0.1) * MaxSpeed, 
                    MathUtil.applyDeadband(-m_joystick.getLeftX(), 0.1) * MaxSpeed,
                    MathUtil.applyDeadband(-m_joystick.getRightX(), 0.1) * MaxAngularRate,
                    m_drivetrain.getRotation3d().toRotation2d()))
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // These are neat but we probably don't need them
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_joystick.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_joystick.getLeftY(), -m_joystick.getLeftX()))
        ));

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

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            m_drivetrain.applyRequest(() -> idle)
        );
    }

    public void periodic() {
        for (VisionMeasurement visionMeasurement : m_limelightSubsystem.getAllLimelightData()) {
            m_drivetrain.addVisionMeasurement(visionMeasurement.m_visionPose, visionMeasurement.m_timeStamp);
        }
    }
}
