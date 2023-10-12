package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private final Compressor compressor = new Compressor(16, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid claw = new DoubleSolenoid(16, PneumaticsModuleType.REVPH, 3, 0);
  
  public Claw() {
    compressor.enableAnalog(90, 120);
  }

  public void openClaw(){
    claw.set(DoubleSolenoid.Value.kForward);
  }
  public void closeClaw(){
    claw.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
  }
}
