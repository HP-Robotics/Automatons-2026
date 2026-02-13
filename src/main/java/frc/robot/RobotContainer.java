// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.GeometryUtil.SphericalCoordinate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.GeometryUtil;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem m_intakeSubsystem = (SubsystemConstants.useIntake) ? new IntakeSubsystem() : null;
    public final ShooterSubsystem m_shooterSubsystem = (SubsystemConstants.useShooter) ? new ShooterSubsystem() : null;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));
        if (SubsystemConstants.useIntake) {
            ControllerConstants.intakeTrigger.whileTrue(m_intakeSubsystem.Intake());
        }

        if (SubsystemConstants.useShooter) {
            ControllerConstants.setShooterTrigger.whileTrue(m_shooterSubsystem.fixedShooter());
            ControllerConstants.stopShooterTrigger.whileTrue(m_shooterSubsystem.stopShooter());
            ControllerConstants.adjustableShooterTrigger.whileTrue(m_shooterSubsystem.adjustableShooter());
            ControllerConstants.magicShooterTrigger.whileTrue(m_shooterSubsystem.magicShooter());
        }

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Rotation2d getAngleToHub() {
        Pose2d pose = drivetrain.getState().Pose;
        Rotation2d angleToHub = new Pose2d(FieldConstants.hub, new Rotation2d()).relativeTo(pose).getTranslation()
                .getAngle();
        return angleToHub;
    }

    public double[] staticShot() {
        Pose2d pose = drivetrain.getState().Pose;
        double distance = pose.getTranslation().getDistance(FieldConstants.hub);
        // Get the hub as a Pose2d (vector representing where it is on the field) -->
        // find the hub in robot relative coordinates --> get the angle with the x axis
        // (robot-facing direction)
        // TODO: factor in turret position compared to robot middle
        Rotation2d angleToHub = getAngleToHub();

        Matrix<N2, N1> staticShot = ShooterConstants.distanceToStaticShot.get(distance);
        
        // turret.pointin(angleToHub); TODO: merge branch to use the turret function
        // hood.setAngle(staticShot.get(1,0));
        m_shooterSubsystem.setSpeed(staticShot.get(0, 0));
        double[] output = {0, 0, 0};
        output[0] = staticShot.get(0, 0); // hood angle?
        output[1] = staticShot.get(0, 1); // wheel speeds?
        output[2] = angleToHub.getRadians(); // turret angle, in radians right now

        return output;
    }

     public void movingShot(double[] staticShot) {
        // Step 2: converting the motor outputs from Step 1 to spherical coordinates
        SphericalCoordinate motorOutputSpherical = new SphericalCoordinate(staticShot[0], Radians.of(staticShot[1]), Radians.of(staticShot[2]));
        // Step 3: convert spherical coordinates to cartesian coordinates
        Translation3d motorOutputCartesian = GeometryUtil.sphericalToCartesian(motorOutputSpherical);
        // Step 4: subtract the driving velocity off of the cartesian static shot
        ChassisSpeeds driveVelocity = drivetrain.getState().Speeds;
        Translation3d driveTranslation = new Translation3d(driveVelocity.vxMetersPerSecond, driveVelocity.vyMetersPerSecond, 0.0);
        Translation3d dynamicShotVector = motorOutputCartesian.minus(driveTranslation);

        SphericalCoordinate dynamicShotSpherical = GeometryUtil.cartesianToSpherical(dynamicShotVector.getX(), dynamicShotVector.getY(), dynamicShotVector.getZ());
        double wheelSpeed = dynamicShotSpherical.magnitude;
        Angle hoodAngle = dynamicShotSpherical.pitch;
        Angle turretAngle = dynamicShotSpherical.yaw;
        // Step 5: set the motors for the wheel speed, hood angle, and turret angle
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
