package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPIDCommand extends CommandBase{
    
    private final Arm arm;
    private final String stage;

    public ArmPIDCommand(Arm arm, String stage){
        this.arm = arm;
        this.stage = stage;
    }

    public void initialize(){

    }
    public void execute(){
        switch(stage){
            case "coneLow":
                arm.coneLow();
                break;
        }
    }
    public void end(boolean interrupted){
        arm.stop();
    }
    public boolean isFinished(){
        return false;
    }
}
