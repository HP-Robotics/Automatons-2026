package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDConstants;

public class IntakeSubsystem extends SubsystemBase {
    TalonFX m_intakeMotor = new TalonFX(MotorIDConstants.intakeSpinMotor);
    TalonFX m_leftMotor = new TalonFX(MotorIDConstants.intakeExtendLeftMotor);
    TalonFX m_rightMotor = new TalonFX(MotorIDConstants.intakeExtendRightMotor);
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeSubsystem");
    Slot0Configs m_intakeConfig = new Slot0Configs();

    private Optional<ExtendProfile> m_currentLeftProfile = Optional.empty();
    private Optional<ExtendProfile> m_currentRightProfile = Optional.empty();

    class ExtendProfile {
        TalonFX m_motor;
        Timer m_timer;
        double m_startTime;
        TrapezoidProfile m_profile;
        TrapezoidProfile.State m_setpoint;
        TrapezoidProfile.State m_goal;

        ExtendProfile(TalonFX motor, double goalPos) {
            m_motor = motor;
            m_timer = new Timer();
            m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakeConstants.extendCruiseVelocity, IntakeConstants.extendAcceleration));
            m_goal = new TrapezoidProfile.State(goalPos, 0);
            m_setpoint = new TrapezoidProfile.State();
        }

        boolean updateMotor() {
            PositionVoltage request = new PositionVoltage(0).withSlot(0);
            double t = m_timer.get();
            m_setpoint = m_profile.calculate(t + 0.020, m_setpoint, m_goal);
            request.Position = m_setpoint.position;
            request.Velocity = m_setpoint.velocity;
            m_motor.setControl(request);

            return m_profile.isFinished(t);
        }
    }

    public IntakeSubsystem() {

        // m_rightMotor.setControl(new Follower(MotorIDConstants.intakeExtendLeftMotor,
        // MotorAlignmentValue.Aligned));
        m_intakeConfig.kP = IntakeConstants.kP;
        m_intakeConfig.kI = IntakeConstants.kI;
        m_intakeConfig.kD = IntakeConstants.kD;

        m_rightMotor.setPosition(0);
        m_leftMotor.setPosition(0);

        var leftMotorConfigs = new TalonFXConfiguration();
        var rightMotorConfigs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive))

                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can
                                // set a
                                // relatively low
                                // stator current limit to help avoid brownouts without
                                // impacting performance.
                                .withStatorCurrentLimit(Amps.of(200))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(200))
                                .withSupplyCurrentLimitEnable(true));

        var leftSlot0Configs = leftMotorConfigs.Slot0;
        leftSlot0Configs.kS = IntakeConstants.leftkS;
        leftSlot0Configs.kV = IntakeConstants.leftkV;
        leftSlot0Configs.kA = IntakeConstants.leftkA;
        leftSlot0Configs.kP = IntakeConstants.leftkP;
        leftSlot0Configs.kI = IntakeConstants.leftkI;
        leftSlot0Configs.kD = IntakeConstants.leftkD;

        m_leftMotor.getConfigurator().apply(leftMotorConfigs);

        var rightSlot0Configs = rightMotorConfigs.Slot0;
        rightSlot0Configs.kS = IntakeConstants.rightkS; // Add 0.25 V output to overcome static friction
        rightSlot0Configs.kV = IntakeConstants.rightkV; // A velocity target of 1 rps results in 0.12 V output
        rightSlot0Configs.kA = IntakeConstants.rightkA; // An acceleration of 1 rps/s requires 0.01 V output
        rightSlot0Configs.kP = IntakeConstants.rightkP; // A position error of 2.5 rotations results in 12 V output
        rightSlot0Configs.kI = IntakeConstants.rightkI; // no output for integrated error
        rightSlot0Configs.kD = IntakeConstants.rightkD; // A velocity error of 1 rps results in 0.1 V output

        m_rightMotor.getConfigurator().apply(rightMotorConfigs);
    }

    public void periodic() {
        table.putValue(("intakeMotorValue"),
                NetworkTableValue.makeDouble(m_intakeMotor.getPosition().getValueAsDouble()));
        table.putValue(("leftMotorValue"),
                NetworkTableValue.makeDouble(m_leftMotor.getPosition().getValueAsDouble()));
        table.putValue(("rightMotorValue"),
                NetworkTableValue.makeDouble(m_rightMotor.getPosition().getValueAsDouble()));

        if (m_currentLeftProfile.isPresent() && m_currentLeftProfile.get().updateMotor()) {
            m_currentLeftProfile = Optional.empty();
        }
        if (m_currentRightProfile.isPresent() && m_currentRightProfile.get().updateMotor()) {
            m_currentRightProfile = Optional.empty();
        }
    }

    public void runIntake(double speed) {
        // m_intakeMotor.set(speed);
    }

    public void stopIntake() {
        // m_intakeMotor.set(0);
    }

    public void extendIntake() {
        m_currentLeftProfile = Optional.of(new ExtendProfile(m_leftMotor, IntakeConstants.leftExtendPosition));
        m_currentRightProfile = Optional.of(new ExtendProfile(m_rightMotor, IntakeConstants.rightExtendPosition));
    }

    public void retractIntake() {
        m_currentLeftProfile = Optional.of(new ExtendProfile(m_leftMotor, IntakeConstants.leftRetractPosition));
        m_currentRightProfile = Optional.of(new ExtendProfile(m_rightMotor, IntakeConstants.rightRetractPosition));
    }

    public Command Intake() {
        return new StartEndCommand(
                () -> {
                    this.runIntake(IntakeConstants.speed);
                    this.extendIntake();
                },
                () -> {
                    this.retractIntake();
                    this.stopIntake();
                }, this);
    }

    public Command StartIntake() {
        return new InstantCommand(
                () -> {
                    this.runIntake(IntakeConstants.speed);
                }, this);
    }

    public Command StopIntake() {
        return new InstantCommand(
                () -> {
                    this.stopIntake();
                }, this);
    }

    public Command ExtendIntake() {
        return new InstantCommand(
                () -> {
                    this.extendIntake();
                }, this);
    }

    public Command RetractIntake() {
        return new InstantCommand(
                () -> {
                    this.retractIntake();
                }, this);
    }
}