package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCMD extends CommandBase {
  private final Claw claw;
  private boolean isOpen = true;

  public ClawCMD(Claw claw) {
    this.claw = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(isOpen){
      claw.openClaw();
    }
    else{
      claw.closeClaw();
    }
    isOpen = !isOpen;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
