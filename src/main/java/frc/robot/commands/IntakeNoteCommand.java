package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Picks up a note with the Intake Subsystem. Command ends when a Note is picked
 * up.
 */

public class IntakeNoteCommand extends Command {

  //
  private IntakeSubsystem m_IntakeSubsystem;

  public IntakeNoteCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setSpeed();
  }

  @Override
  public void execute() {
    // nothing to do yet
  }

  public boolean isFinished() {

    if (m_IntakeSubsystem.hasNote()) return true;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
  }

}
