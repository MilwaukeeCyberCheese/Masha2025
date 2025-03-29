package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ChuteDrop extends Command {

  private final ChuteSubsystem m_chute;
  private final ClimberSubsystem m_climber;

  /**
   * Drops the chute if the climber is down, otherwise lowers the climber and then drops the chute
   *
   * @param chute
   * @param climber
   */
  public ChuteDrop(ChuteSubsystem chute, ClimberSubsystem climber) {
    m_chute = chute;
    m_climber = climber;
    addRequirements(m_chute, m_climber);
  }

  @Override
  public void execute() {
    if (m_climber.isDown()) {
      m_chute.drop();
    } else {
      m_climber.downSlow();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.inactive();
  }

  @Override
  public boolean isFinished() {
    return m_chute.getState() == ChuteSubsystem.ChuteState.DOWN;
  }
}
