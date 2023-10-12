package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmMode extends CommandBase{
    private final Arm arm;
    private boolean isCone;

    public ArmMode(Arm arm){
        this.arm = arm;
    }

    public void initialize(){}

    public void execute(){
        arm.togleMode();
    }

    public void end(boolean interrupted){}

    public boolean isFinished(){
        return true;
    }

}
