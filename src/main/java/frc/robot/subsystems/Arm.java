package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1;
    private final CANSparkMax raiseMotor2;
    private final CANSparkMax extendMotor;

    private final RelativeEncoder raiseEncoder1;
    private final RelativeEncoder raiseEncoder2;
    private final RelativeEncoder extendEncoder;

    private boolean isCone;

    private final PIDController armExtendPID;
    private final PIDController armRaisePID1;
    private final PIDController armRaisePID2;

    public Arm(){
        this.raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
        this.raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
        this.extendMotor = new CANSparkMax(15, MotorType.kBrushless);

        raiseMotor2.setInverted(true);

        raiseEncoder1 = raiseMotor1.getEncoder();
        raiseEncoder2 = raiseMotor2.getEncoder();
        extendEncoder = extendMotor.getEncoder();

        isCone = false;//Change if initial mode is different

        armExtendPID = new PIDController(0.44891030029999945400000000000, 0, 0);
        armRaisePID1 = new PIDController(0.140600000000000, 0, 0);
        armRaisePID2 = new PIDController(0.140600000000000, 0, 0);
    }

    public void periodic(){

    }

    public double getRaise1Position(){
        return raiseEncoder1.getPosition();
    }
    public double getRaise2Position(){
        return raiseEncoder2.getPosition();
    }
    public double getExtendPosition(){
        return extendEncoder.getPosition();
    }
    public void setIsCone(boolean isCone){
        this.isCone = isCone;
    }
    public boolean getIsCone(){
        return isCone;
    }
    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
        extendMotor.stopMotor();
    }

    public void setPreset(String stage){
        new Thread(() -> {
            extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get(stage)));
        }).start();
        new Thread(() -> {
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
        new Thread(() -> {
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
    }
}