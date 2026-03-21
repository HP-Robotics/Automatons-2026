package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class WiggleCommand extends Command {
  private final IntakeSubsystem m_subsystem;
  private Timer m_timer = new Timer();
  private boolean m_extended;

  public WiggleCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // m_subsystem.slowExtend();
    m_subsystem.extendIntake();
    m_extended = true;
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(IntakeConstants.wiggleTime)) {
      if (m_extended) {
        m_subsystem.wiggleIntake();
        m_timer.reset();
        m_extended = false;
      } else {
        m_subsystem.extendIntake();
        m_timer.reset();
        m_extended = true;
      }
    }
  }

  @Override
  public void end(boolean interupted) {
    m_subsystem.fastExtend();
    m_subsystem.extendIntake();
  }
}