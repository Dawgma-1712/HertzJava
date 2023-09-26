package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmJoystickCommand extends CommandBase{
    private Arm arm;
    private Supplier<Double> extendValue, raiseValue;

    public ArmJoystickCommand(Arm arm, Supplier<Double> extendValue, Supplier<Double> raiseValue) {
        this.arm = arm;
        this.extendValue = extendValue;
        this.raiseValue = raiseValue;
        addRequirements(arm);
    }

    public void initialize() {}

    @Override
    public void execute() {
        double extend = extendValue.get();
        double raise = raiseValue.get();
        arm.manualArm(extend, raise);

  }

    @Override
    public void end(boolean interrupted) {

  }

    @Override
    public boolean isFinished() {
    return false;
  }
}
