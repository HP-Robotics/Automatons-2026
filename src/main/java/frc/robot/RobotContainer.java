// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
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
	private SendableChooser<Command> autoChooser = null;
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
	public double m_shotIsLegalFrameCounter = ShooterConstants.shotIsLegalBonusFrames;
	public double m_dynamicWheelSpeed;
	public double m_dynamicHoodAngle;
	public double m_dynamicTurretAngle;
	public boolean m_useNetworkTableShooter = false;
	public boolean m_trenchOverride = false;

	public RobotContainer() {
		DataLogManager.start();

		// Another option that allows you to specify the default auto by its name
		// autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
		configurePathPlannerCommands();

		if (SubsystemConstants.useDrive) {
			autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("Auto Chooser", autoChooser);
		}
		configureBindings();

		SmartDashboard.putData("field", m_field);
		// m_velocityInterpolator.draw("/media/sda1/draws/xVelcoty.png",
		// 1000,
		// 1000, 45, 90, 2.7, 0, 0, 0, 6);
		// m_velocityInterpolator.draw("/media/sda1/draws/yVelcoty.png",
		// 1000,
		// 1000, 45, 90, 2.7, 0, 1, 0, 10);
	}

	private void configurePathPlannerCommands() {
		if (SubsystemConstants.useClimber) {
			new EventTrigger("Climber up").onTrue(m_climberSubsystem.ClimbUp());
			new EventTrigger("Climber down").onTrue(m_climberSubsystem.ClimbDown());
			NamedCommands.registerCommand("CommandClimberUp", m_climberSubsystem.ClimbUp().asProxy());
			NamedCommands.registerCommand("CommandClimberDown", m_climberSubsystem.ClimbDown().asProxy());
		}

		new EventTrigger("Shoot").onTrue(startShooter());
		NamedCommands.registerCommand("CommandShoot",
				new InstantCommand(() -> CommandScheduler.getInstance().schedule(startShooter())));

		if (SubsystemConstants.useHopper) {
			new EventTrigger("CancelShooter").onTrue(cancelHopper());
			NamedCommands.registerCommand("CommandCancelShooter", cancelHopper().asProxy());
		}

		if (SubsystemConstants.useIntake) {
			new EventTrigger("Start Intake").onTrue(m_intakeSubsystem.StartIntake());
			new EventTrigger("Stop Intake").onTrue(m_intakeSubsystem.StopIntake());
			new EventTrigger("Extend Intake").onTrue(m_intakeSubsystem.ExtendIntake());
			new EventTrigger("Retract Intake").onTrue(m_intakeSubsystem.RetractIntake());
			NamedCommands.registerCommand("Extend Intake", m_intakeSubsystem.ExtendIntake().asProxy());
			NamedCommands.registerCommand("Wiggle Intake",
					new InstantCommand(() -> CommandScheduler.getInstance()
							.schedule(new WiggleCommand(m_intakeSubsystem).asProxy())));
		}
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
					m_drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
							.withDriveRequestType(DriveRequestType.Velocity)
							.withDeadband(0.01 * m_maxSpeed)
							.withRotationalDeadband(0.01 * m_maxAngularRate)
							.withVelocityX(-ControllerConstants.m_leftAxisY.getAsDouble() * m_maxSpeed)
							.withVelocityY(-ControllerConstants.m_leftAxisX.getAsDouble() * m_maxSpeed)

							.withRotationalRate(
									-ControllerConstants.m_rightAxisX.getAsDouble() * m_maxAngularRate)));
			RobotModeTriggers.disabled().whileTrue(
					m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

			ControllerConstants.trenchOrientation.whileTrue(
					// Drivetrain will execute this command periodically
					m_drivetrain.applyRequest(
							() -> {
								double targetAngle = 0;
								double robotAngle = MathUtil
										.inputModulus(m_drivetrain.getState().Pose.getRotation().getDegrees(), 0, 360);
								double trenchError = m_drivetrain.getCloseTrench() - m_drivetrain.getPose().getY();
								if (robotAngle % 90 > 45) {
									targetAngle = robotAngle + 90 - (robotAngle % 90);
								} else {
									targetAngle = robotAngle - (robotAngle % 90);
								}
								return new SwerveRequest.FieldCentricFacingAngle()
										.withDriveRequestType(DriveRequestType.Velocity)
										.withDeadband(0.1 * m_maxSpeed)
										.withHeadingPID(3.5, 0.0, 0.0)
										.withVelocityX(-ControllerConstants.m_leftAxisY.getAsDouble() * m_maxSpeed)
										.withVelocityY(MathUtil.clamp(trenchError * DriveConstants.trenchAutoP,
												-m_maxSpeed, m_maxSpeed))
										.withTargetDirection(Rotation2d.fromDegrees(targetAngle));
							}))
					.onTrue(new InstantCommand(() -> m_trenchOverride = true))
					.onFalse(new InstantCommand(() -> m_trenchOverride = false));

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
			ControllerConstants.intakeTrigger.onTrue(m_intakeSubsystem.ToggleIntake());
			ControllerConstants.yuckTrigger.whileTrue(m_intakeSubsystem.Yuck());
			ControllerConstants.wiggleTrigger.whileTrue(new WiggleCommand(m_intakeSubsystem));
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
					m_hopperSubsystem.MagicHopper(this::readyToShoot));
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
			// ControllerConstants.calibrateTurretTrigger.whileTrue(m_turretSubsystem.Calibrate());
			ControllerConstants.ShooterNetworkTablesModeTrigger.whileTrue(new SequentialCommandGroup(
					new InstantCommand(() -> m_turretSubsystem.getFromNetworkTables()),
					m_turretSubsystem.RotateTurret()));

		}

		ControllerConstants.magicShooterTrigger.onTrue(startShooter());
		if (SubsystemConstants.useHopper) {
			ControllerConstants.magicShooterTrigger.onFalse(cancelHopper());
			// TODO:allow canceling if there is no hopper
		}

		ControllerConstants.ShooterNetworkTablesModeTrigger
				.onTrue(new InstantCommand(() -> m_useNetworkTableShooter = true))
				.onFalse(new InstantCommand(() -> m_useNetworkTableShooter = false));
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

		Matrix<N2, N1> staticShot;
		if (target == FieldConstants.blueHub || target == FieldConstants.redHub) {
			staticShot = ShooterConstants.distanceToStaticShot.get(distance);
		} else {
			staticShot = ShooterConstants.distanceToStaticLobShot.get(distance);
		}
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
			m_shotIsLegalFrameCounter++;
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
			m_shotIsLegalFrameCounter++;
			m_shotIsLegal = false;
			return;
		} else {
			m_shotIsLegalFrameCounter = 0;
		}
		m_dynamicWheelSpeed = dynamicShotMotorOutputs.get()[0];
		m_dynamicHoodAngle = dynamicShotMotorOutputs.get()[1];
		m_dynamicTurretAngle = dynamicShotSpherical.yaw.in(Degrees);
		// Step 5: set the motors for the wheel speed, hood angle, and turret angle

		m_table.putValue("dynamicWheelSpeed", NetworkTableValue.makeDouble(m_dynamicWheelSpeed));
		m_table.putValue("dynamicHoodAngle", NetworkTableValue.makeDouble(m_dynamicHoodAngle));
		m_table.putValue("dynamiCurretAngle", NetworkTableValue.makeDouble(m_dynamicTurretAngle));

		m_shotIsLegal = true;
	}

	public void calibrateTurret() {
		if (m_turretSubsystem != null) {
			m_turretSubsystem.resetMotorEncoders();
		}
	}

	public Command startShooter() {
		ParallelCommandGroup output = new ParallelCommandGroup();
		if (SubsystemConstants.useShooter) {
			output.addCommands(m_shooterSubsystem.MagicShooter(this::getMagicShooterSpeed));
		}
		if (SubsystemConstants.useTurret) {
			output.addCommands(m_turretSubsystem.MagicTurret(this::getMagicTurretAngle));
		}
		if (SubsystemConstants.useHood) {
			output.addCommands(m_hoodSubsystem.MagicHood(this::getMagicHoodPosition));
		}
		if (SubsystemConstants.useHopper) {
			output.addCommands(m_hopperSubsystem.MagicHopper(this::readyToShoot));
		}
		return output;

	}

	public Command cancelHopper() {
		return new InstantCommand(
				() -> {
					Command current = m_hopperSubsystem.getCurrentCommand();
					if (current != null) {
						current.cancel();
					}
				}

		);

	}

	public Command cancelHood() {
		return new InstantCommand(
				() -> {
					Command current = m_hoodSubsystem.getCurrentCommand();
					if (current != null) {
						current.cancel();
					}
				});
	}

	public Command getAutonomousCommand() {
		if (SubsystemConstants.useDrive) {
			return autoChooser.getSelected();
		} else {
			return new WaitCommand(0);
		}
	}

	public void periodic() {
		if (!SubsystemConstants.useDrive) {
			return;
		}
		double rot = m_drivetrain.getState().Pose.getRotation().getDegrees();
		m_limelightSubsystem.updateRobotOrientation(rot);

		for (VisionMeasurement visionMeasurement : m_limelightSubsystem.getAllLimelightData()) {
			m_drivetrain.addVisionMeasurement(visionMeasurement.m_visionPose, visionMeasurement.m_timeStamp,
					visionMeasurement.m_stdevsVector);
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

			if (SubsystemConstants.useShooter && SubsystemConstants.useTurret) {
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
				m_table.putValue("readyToShoot", NetworkTableValue.makeBoolean(readyToShoot()));
				m_table.putValue("legalFrames", NetworkTableValue.makeDouble(m_shotIsLegalFrameCounter));
			}
		}
		m_field.setRobotPose(m_drivetrain.getState().Pose);
		m_table.putValue("timeBeforeActiveShift", NetworkTableValue.makeInteger((int)Math.floor(timeBeforeShift())));
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

	public double getMagicTurretAngle() {
		if (m_shotIsLegalFrameCounter < ShooterConstants.shotIsLegalBonusFrames) {
			return m_dynamicTurretAngle;
		} else {
			return m_turretToHub;
		}
	}

	public double getMagicHoodPosition() {
		if (m_trenchOverride) {
			return HoodConstants.hoodBottom;
		} else if (m_shotIsLegalFrameCounter < ShooterConstants.shotIsLegalBonusFrames) {
			return m_dynamicHoodAngle;
		} else {
			return m_staticHoodPosition;
		}
	}

	public double getMagicShooterSpeed() {
		double fudge = 0.0;
		if (SubsystemConstants.useTurret) {
			fudge = ShooterConstants.turretAngleFudge.get(Math.abs(m_turretSubsystem.getAngle().in(Degrees)));
		}

		m_table.putValue("turretFudge", NetworkTableValue.makeDouble(fudge));
		if (m_shotIsLegalFrameCounter < ShooterConstants.shotIsLegalBonusFrames) {
			return m_dynamicWheelSpeed * (1.0);
		} else {
			return m_staticWheelSpeed * (1.0);
		}
	}

	public boolean readyToShoot() {
		return ((!SubsystemConstants.useHood || (m_hoodSubsystem.isHoodAimed() && !m_trenchOverride))
				&& (!SubsystemConstants.useShooter
						|| m_shooterSubsystem.atSpeed(ShooterConstants.shooterErrorThreshold))
				&& (!SubsystemConstants.useTurret || m_turretSubsystem.atPosition() || m_useNetworkTableShooter)
				&& (m_useNetworkTableShooter || m_shotIsLegalFrameCounter < ShooterConstants.shotIsLegalBonusFrames));
	}

	public double timeBeforeShift() {
		double matchTime = DriverStation.getMatchTime();
		if (matchTime > 130) {
			// Transition shift, hub is active.
			return matchTime - 130;
		} else if (matchTime > 105) {
			// Shift 1
			return matchTime - 105;
		} else if (matchTime > 80) {
			// Shift 2
			return matchTime - 80;
		} else if (matchTime > 55) {
			// Shift 3
			return matchTime - 55;
		} else if (matchTime > 30) {
			// Shift 4
			return matchTime - 30;
		} else {
			// End game, hub always active.
			return matchTime;
		}
	}
}