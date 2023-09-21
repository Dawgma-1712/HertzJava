package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmMode extends CommandBase{
    private final Arm arm;
    private final boolean isCone;

    public ArmMode(Arm arm, boolean isCone){
        this.arm = arm;
        this.isCone = isCone;
    }

    public void initialize(){

    }
    public void execute(){
        arm.setIsCone(isCone);
    }
    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return arm.getIsCone() == isCone;
    }

}
