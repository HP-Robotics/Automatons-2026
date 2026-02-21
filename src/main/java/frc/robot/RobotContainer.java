// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.GeometryUtil.SphericalCoordinate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;

import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class RobotContainer {
	private double m_maxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
																							// speed
	private double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
																						// max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(m_maxSpeed * 0.1).withRotationalDeadband(m_maxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	// private final SwerveRequest.SwerveDriveBrake brake = new
	// SwerveRequest.SwerveDriveBrake();
	// private final SwerveRequest.PointWheelsAt point = new
	// SwerveRequest.PointWheelsAt();

	private final Telemetry m_logger = new Telemetry(m_maxSpeed);

	private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();

	// TODO: Make these optionals?
	public final CommandSwerveDrivetrain m_drivetrain = (SubsystemConstants.useDrive)
			? TunerConstants.createDrivetrain()
			: null;
	public final IntakeSubsystem m_intakeSubsystem = (SubsystemConstants.useIntake) ? new IntakeSubsystem() : null;
	public final ShooterSubsystem m_shooterSubsystem = (SubsystemConstants.useShooter) ? new ShooterSubsystem() : null;
	public final TurretSubsystem m_turretSubsystem = (SubsystemConstants.useTurret) ? new TurretSubsystem() : null;
	public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
	public final ClimberSubsystem m_climberSubsystem = (SubsystemConstants.useClimber) ? new ClimberSubsystem() : null;
	public final HoodSubsystem m_hoodSubsystem = SubsystemConstants.useHood ? new HoodSubsystem() : null;

	public final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Robot");
	StructPublisher<Pose2d> m_posePublisher = m_table.getStructTopic("targetPose", Pose2d.struct).publish();
	StructPublisher<Pose2d> m_turretPosePublisher = m_table.getStructTopic("turretPose", Pose2d.struct)
			.publish();

	StructPublisher<Translation3d> m_staticShotVelocityPublisher = m_table
			.getStructTopic("staticShotVelocity", Translation3d.struct).publish();
	StructPublisher<Translation3d> m_dynamicShotVelocityPublisher = m_table
			.getStructTopic("dynamicShotVelocity", Translation3d.struct).publish();
	StructPublisher<Translation3d> m_robotVelocityPublisher = m_table
			.getStructTopic("robotVelocity", Translation3d.struct).publish();

	public TriangleInterpolator m_velocityInterpolator = new TriangleInterpolator(2,
			Filesystem.getDeployDirectory().getAbsolutePath().concat("/velocityTriangles.json"),
			Filesystem.getDeployDirectory().getAbsolutePath().concat("/velocityPoints.json"));

	public TriangleInterpolator m_motorOutputInterpolator = new TriangleInterpolator(2,
			Filesystem.getDeployDirectory().getAbsolutePath().concat("/motorOutputTriangles.json"),
			Filesystem.getDeployDirectory().getAbsolutePath().concat("/motorOutputPoints.json"));

	public double m_turretToHub;
	public double m_staticWheelSpeed;
	public double m_staticHoodPosition;
	public Translation2d m_shootingTarget;
	public Field2d m_field = new Field2d();
	public boolean m_shotIsLegal = false;
	public double m_dynamicWheelSpeed;
	public double m_dynamicHoodAngle;
	public double m_dynamicTurretAngle;

	public RobotContainer() {
		configureBindings();
		SmartDashboard.putData("field", m_field);
		// m_velocityInterpolator.draw("/media/sda1/draws/xVelcoty.png",
		// 1000,
		// 1000, 45, 90, 2.7, 0, 0, 0, 6);
		// m_velocityInterpolator.draw("/media/sda1/draws/yVelcoty.png",
		// 1000,
		// 1000, 45, 90, 2.7, 0, 1, 0, 10);
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();

		// joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
		// joystick.b().whileTrue(drivetrain.applyRequest(
		// () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
		// -joystick.getLeftX()))));

		if (SubsystemConstants.useDrive) {
			m_drivetrain.setDefaultCommand(
					// Drivetrain will execute this command periodically
					m_drivetrain.applyRequest(
							() -> m_drivetrain.applySetpointGenerator(ChassisSpeeds.fromFieldRelativeSpeeds(
									MathUtil.applyDeadband(ControllerConstants.m_leftAxisY.getAsDouble(), 0.1)
											* m_maxSpeed,
									MathUtil.applyDeadband(ControllerConstants.m_leftAxisX.getAsDouble(), 0.1)
											* m_maxSpeed,
									MathUtil.applyDeadband(-ControllerConstants.m_rightAxisX.getAsDouble(), 0.1)
											* m_maxAngularRate,
									m_drivetrain.getRotation3d().toRotation2d()))));
			RobotModeTriggers.disabled().whileTrue(
					m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

			// These are neat but we probably don't need them
			// joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
			// m_joystick.b().whileTrue(m_drivetrain.applyRequest(
			// () -> point.withModuleDirection(new Rotation2d(-m_joystick.getLeftY(),
			// -m_joystick.getLeftX()))));

			// // Run SysId routines when holding back/start and X/Y.
			// // Note that each routine should be run exactly once in a single log.
			ControllerConstants.sysIdDynamicForward.whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
			ControllerConstants.sysIdDynamicReverse.whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
			ControllerConstants.sysIdQuasistaticForward.whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
			ControllerConstants.sysIdQuasistaticReverse.whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));
			// Reset the field-centric heading on left bumper press.
			ControllerConstants.setFieldCentricTrigger.onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
			m_drivetrain.registerTelemetry(m_logger::telemeterize);
		}
		if (SubsystemConstants.useIntake) {
			ControllerConstants.intakeTrigger.toggleOnTrue(m_intakeSubsystem.Intake());
		}

		if (SubsystemConstants.useShooter) {
			// ControllerConstants.setShooterTrigger.whileTrue(m_shooterSubsystem.fixedShooter());
			// ControllerConstants.stopShooterTrigger.whileTrue(m_shooterSubsystem.stopShooter());
			ControllerConstants.ShooterNetworkTablesModeTrigger.whileTrue(m_shooterSubsystem.AdjustableShooter());
			// ControllerConstants.magicShooterTrigger.whileTrue(m_shooterSubsystem.magicShooter());
		}

		if (SubsystemConstants.useHopper) {
			ControllerConstants.manualHopperTrigger.whileTrue(m_hopperSubsystem.RunHopper());
		}
		if (SubsystemConstants.useHopper && SubsystemConstants.useShooter) {

			ControllerConstants.ShooterNetworkTablesModeTrigger.whileTrue(
					new ParallelCommandGroup(m_shooterSubsystem.AdjustableShooter(),
							new SequentialCommandGroup(new WaitUntilCommand(m_shooterSubsystem::atSpeed),
									m_hopperSubsystem.RunHopper())));
			// First, spin up the shooter
			// Wait until shoot is spinning
			// Run hopper
		}

		if (SubsystemConstants.useClimber) {
			ControllerConstants.climbUpTrigger.whileTrue(m_climberSubsystem.ClimbUp());
			ControllerConstants.climbDownTrigger.whileTrue(m_climberSubsystem.ClimbDown());
			ControllerConstants.calibrateClimberTrigger
					.whileTrue(m_climberSubsystem.clearCallibration().andThen(m_climberSubsystem.Calibrate()));
		}

		if (SubsystemConstants.useHood) {
			ControllerConstants.ShooterNetworkTablesModeTrigger.whileTrue(m_hoodSubsystem.hoodFromNetworkTables());
		}

		if (SubsystemConstants.useTurret) {
			// ControllerConstants.runTurretTrigger.whileTrue(new
			// StartEndCommand(m_turretSubsystem::runTurret,
			// m_turretSubsystem::stopTurret, m_turretSubsystem));
			ControllerConstants.calibrateTurretTrigger.whileTrue(m_turretSubsystem.Calibrate());
			ControllerConstants.ShooterNetworkTablesModeTrigger.whileTrue(m_turretSubsystem.RotateTurret());

		}

		if (SubsystemConstants.useShooter && SubsystemConstants.useTurret && SubsystemConstants.useHood) {
			ControllerConstants.runStaticShotTrigger.whileTrue(
					new ParallelCommandGroup(
							new RunCommand(() -> {
								if (m_shotIsLegal) {
									m_shooterSubsystem.setVelocity(m_dynamicWheelSpeed);

									m_hoodSubsystem.setHood(m_dynamicHoodAngle);
									m_turretSubsystem.setTargetPosition(m_dynamicTurretAngle);
									m_turretSubsystem.rotateTurretToTarget();
								}
							}, m_shooterSubsystem, m_hoodSubsystem).finallyDo(m_shooterSubsystem::stopMotor),
							new SequentialCommandGroup(
									new WaitUntilCommand(m_shooterSubsystem::atSpeed),
									m_hopperSubsystem.RunHopper())));
		}
	}

	public double[] calculateStaticShot(Translation2d target, Pose2d robotPose) {
		// Get the hub as a Pose2d (vector representing where it is on the field) -->
		// find the hub in robot relative coordinates --> get the angle with the x axis
		// (robot-facing direction)
		Pose2d robotToHub = new Pose2d(target, new Rotation2d()).relativeTo(robotPose);
		Translation2d turretToHub = robotToHub.getTranslation().minus(TurretConstants.centerPosition);

		// Rotation2d robotAngleToHub = turretToHub.getAngle();
		// double distance = turretToHub.getNorm();
		// m_turretToHub = robotAngleToHub.getDegrees();
		Translation2d turretPosition = TurretConstants.centerPosition.rotateBy(robotPose.getRotation())
				.plus(robotPose.getTranslation());
		Pose2d alternateTurretToHub = new Pose2d(target, new Rotation2d())
				.relativeTo(new Pose2d(turretPosition, robotPose.getRotation()));
		Rotation2d robotAngleToHub = alternateTurretToHub.getTranslation().getAngle();
		double distance = alternateTurretToHub.getTranslation().getNorm();
		m_turretToHub = robotAngleToHub.getDegrees();

		m_table.putValue("turretToHub", NetworkTableValue.makeDouble(m_turretToHub));
		m_table.putValue("distance", NetworkTableValue.makeDouble(distance));
		m_posePublisher.set(new Pose2d(target, new Rotation2d()));
		m_turretPosePublisher.set(new Pose2d(turretPosition,
				new Rotation2d(m_turretSubsystem.getAngle().plus(robotPose.getRotation().getMeasure()))));

		Matrix<N2, N1> staticShot = ShooterConstants.distanceToStaticShot.get(distance);
		m_staticWheelSpeed = staticShot.get(0, 0);
		m_staticHoodPosition = staticShot.get(1, 0);

		m_table.putValue("staticWheelSpeed", NetworkTableValue.makeDouble(m_staticWheelSpeed));
		m_table.putValue("staticHoodPosition", NetworkTableValue.makeDouble(m_staticHoodPosition));

		double[] output = new double[3];
		output[0] = m_staticWheelSpeed;
		output[1] = m_staticHoodPosition;
		output[2] = m_turretToHub;
		return output;
	}

	public void movingShot(double[] staticShot) {
		// Step 2: converting the motor outputs from Step 1 to spherical coordinates
		Optional<double[]> staticShotVelocities = m_velocityInterpolator.getTriangulatedOutput(staticShot[0],
				staticShot[1]);
		if (staticShotVelocities.isEmpty()) {
			m_shotIsLegal = false;
			return;
		}
		double totalVelocity = Math
				.sqrt(Math.pow(staticShotVelocities.get()[0], 2) + Math.pow(staticShotVelocities.get()[1], 2));
		double pitchAngle = Math.atan2(staticShotVelocities.get()[1], staticShotVelocities.get()[0]);
		SphericalCoordinate motorOutputSpherical = new SphericalCoordinate(
				totalVelocity,
				Radians.of(pitchAngle),
				Degrees.of(staticShot[2]));
		m_table.putValue("staticShotXVelocity", NetworkTableValue.makeDouble(staticShotVelocities.get()[0]));
		m_table.putValue("staticShotYVelocity", NetworkTableValue.makeDouble(staticShotVelocities.get()[1]));
		m_table.putValue("totalVelocity", NetworkTableValue.makeDouble(totalVelocity));
		m_table.putValue("staticShotPitchAngle", NetworkTableValue.makeDouble(pitchAngle));
		m_table.putValue("staticYawAngle", NetworkTableValue.makeDouble(staticShot[2]));

		// Step 3: convert spherical coordinates to cartesian coordinates
		Translation3d motorOutputCartesian = GeometryUtil.sphericalToCartesian(motorOutputSpherical);
		// Step 4: subtract the driving velocity off of the cartesian static shot
		ChassisSpeeds driveVelocity = m_drivetrain.getState().Speeds;
		Translation3d driveTranslation = new Translation3d(driveVelocity.vxMetersPerSecond,
				driveVelocity.vyMetersPerSecond, 0.0);
		Translation3d dynamicShotVector = motorOutputCartesian.minus(driveTranslation);
		SphericalCoordinate dynamicShotSpherical = GeometryUtil.cartesianToSpherical(dynamicShotVector.getX(),
				dynamicShotVector.getY(), dynamicShotVector.getZ());
		Optional<double[]> dynamicShotMotorOutputs = m_motorOutputInterpolator.getTriangulatedOutput(
				dynamicShotSpherical.magnitude * Math.cos(dynamicShotSpherical.pitch.in(Radians)),
				dynamicShotSpherical.magnitude * Math.sin(dynamicShotSpherical.pitch.in(Radians)));
		m_table.putValue("dynamicShotSphericalMagnitude", NetworkTableValue.makeDouble(dynamicShotSpherical.magnitude));
		m_staticShotVelocityPublisher.set(motorOutputCartesian);
		m_dynamicShotVelocityPublisher.set(dynamicShotVector);
		m_robotVelocityPublisher.set(driveTranslation);
		if (dynamicShotMotorOutputs.isEmpty()) {
			m_shotIsLegal = false;
			return;
		}
		m_dynamicWheelSpeed = dynamicShotMotorOutputs.get()[0];
		m_dynamicHoodAngle = dynamicShotMotorOutputs.get()[1];
		m_dynamicTurretAngle = dynamicShotSpherical.yaw.in(Degrees);
		// Step 5: set the motors for the wheel speed, hood angle, and turret angle

		m_table.putValue("wheelSpeed", NetworkTableValue.makeDouble(m_dynamicWheelSpeed));
		m_table.putValue("hoodAngle", NetworkTableValue.makeDouble(m_dynamicHoodAngle));
		m_table.putValue("turretAngle", NetworkTableValue.makeDouble(m_dynamicTurretAngle));

		m_shotIsLegal = true;
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
		if (!SubsystemConstants.useDrive) {
			return;
		}
		double rot = m_drivetrain.getState().Pose.getRotation().getDegrees();
		m_limelightSubsystem.updateRobotOrientation(rot);

		for (VisionMeasurement visionMeasurement : m_limelightSubsystem.getAllLimelightData()) {
			m_drivetrain.addVisionMeasurement(visionMeasurement.m_visionPose, visionMeasurement.m_timeStamp);
		}
		if (DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Red) {
				if (m_drivetrain.getState().Pose.getX() > FieldConstants.redAllianceZoneX) {
					m_shootingTarget = FieldConstants.redHub;
				} else {
					if (m_drivetrain.getState().Pose.getY() > FieldConstants.centerLineY) {
						m_shootingTarget = FieldConstants.redOutpostSide;
					} else {
						m_shootingTarget = FieldConstants.redDepotSide;
					}
				}
			} else {
				if (m_drivetrain.getState().Pose.getX() < FieldConstants.blueAllianceZoneX) {
					m_shootingTarget = FieldConstants.blueHub;
				} else {
					if (m_drivetrain.getState().Pose.getY() < FieldConstants.centerLineY) {
						m_shootingTarget = FieldConstants.blueOutpostSide;
					} else {
						m_shootingTarget = FieldConstants.blueDepotSide;
					}
				}

			}

			if (Constants.SubsystemConstants.useShooter) {
				movingShot(
						calculateStaticShot(
								m_shootingTarget,
								m_drivetrain.getState().Pose));
				// .plus(
				// new Transform2d(
				// new Translation2d(
				// m_drivetrain.getState().Speeds.vxMetersPerSecond,
				// m_drivetrain.getState().Speeds.vyMetersPerSecond)
				// .times(ShooterConstants.lookAheadTime),
				// new Rotation2d()))));

				m_table.putValue("shotIsLegal", NetworkTableValue.makeBoolean(m_shotIsLegal));
			}
		}
		m_field.setRobotPose(m_drivetrain.getState().Pose);

	}

	public void CalibrateClimber() {
		if (SubsystemConstants.useClimber) {
			CommandScheduler.getInstance().schedule(m_climberSubsystem.Calibrate());
		}
	}

	public void CalibrateHood() {
		if (SubsystemConstants.useHood) {
			CommandScheduler.getInstance().schedule(m_hoodSubsystem.Calibrate());
		}
	}

	public void CalibrateIntake() {
		if (SubsystemConstants.useIntake) {
			CommandScheduler.getInstance().schedule(m_intakeSubsystem.Calibrate());
		}
	}
}
