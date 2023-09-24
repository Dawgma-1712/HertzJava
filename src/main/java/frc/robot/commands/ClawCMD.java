package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCMD extends CommandBase {
  private final Claw claw;
  private boolean isOpen;

  public ClawCMD(Claw claw, boolean isOpen) {
    this.claw = claw;
    this.isOpen = isOpen;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    claw.setIsOpen(isOpen);
  }

  @Override
  public void execute() {
    claw.setClaw();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
