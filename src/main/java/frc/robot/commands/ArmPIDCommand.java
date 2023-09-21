package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
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
        arm.setPreset(stage);
    }
    public void end(boolean interrupted){
        arm.stop();
    }
    public boolean isFinished(){
        if(stage.equals("high") || stage.equals("mid")){
            if(arm.getIsCone()){
                switch(stage){
                    case "high":
                        return Math.abs(OperatorConstants.armExtendPresets.get("coneHigh") - arm.getExtendPosition()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("coneHigh") - arm.getRaise1Position()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("coneHigh") - Math.abs(arm.getRaise2Position())) < 3;
                    case "low":
                        return Math.abs(OperatorConstants.armExtendPresets.get("coneMid") - arm.getExtendPosition()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("coneMid") - arm.getRaise1Position()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("coneMid") - Math.abs(arm.getRaise2Position())) < 3;
                }
            }
            else{
                switch(stage){
                    case "high":
                        return Math.abs(OperatorConstants.armExtendPresets.get("cubeHigh") - arm.getExtendPosition()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("cubeHigh") - arm.getRaise1Position()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("cubeHigh") - Math.abs(arm.getRaise2Position())) < 3;
                    case "low":
                        return Math.abs(OperatorConstants.armExtendPresets.get("cubeMid") - arm.getExtendPosition()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("cubeMid") - arm.getRaise1Position()) < 3 && 
                                Math.abs(OperatorConstants.armRaisePresets.get("cubeMid") - Math.abs(arm.getRaise2Position())) < 3;
                }
            }
        }
        else{
            return Math.abs(OperatorConstants.armExtendPresets.get(stage) - arm.getExtendPosition()) < 3 && 
            Math.abs(OperatorConstants.armRaisePresets.get(stage) - arm.getRaise1Position()) < 3 && 
            Math.abs(OperatorConstants.armRaisePresets.get(stage) - Math.abs(arm.getRaise2Position())) < 3;
        }
        return false;
    }
}
