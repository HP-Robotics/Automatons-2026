package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterMotor1 = new TalonFX(MotorIDConstants.shooterMotor1);
    TalonFX shooterMotor2 = new TalonFX(MotorIDConstants.shooterMotor2);
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterSubsystem");

    public ShooterSubsystem() {

    }
    // shooter modes: magic mode, fixed speed, network tables, stopped, idle

    public void setSpeed(double speed) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    public void stopWheel() {
        setSpeed(0);

    }

    public void idleShooter() {
        setSpeed(ShooterConstants.idleSpeed);

    }

    public void networktablesSpeed() {
        double speed = table.getEntry("shooterSpeed").getDouble(ShooterConstants.idleSpeed);
        setSpeed(speed);
    }

    public void magicSpeed() {
        double speed = ShooterConstants.idleSpeed;
        // TODO: add the magic
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    public Command fixedShot() {
        return new StartEndCommand(() -> {
            setSpeed(ShooterConstants.shootingSpeed);

        },
                this::idleShooter,
                this);

    }

    public Command stopShooter() {

        return new InstantCommand(this::stopWheel, this);
    }

    public Command idle() {

        return new StartEndCommand(this::idleShooter,
                this::stopWheel, this);
    }

    public Command adjustableSpeed() {
        return new RunCommand(this::networktablesSpeed, this).finallyDo(this::stopWheel);
    }

    public Command magicShot() {
        return new StartEndCommand(this::magicSpeed,
                this::idleShooter, this);
    }

}
