package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax extendMotor = new CANSparkMax(15, MotorType.kBrushless);

    private final RelativeEncoder raiseEncoder1 = raiseMotor1.getEncoder();
    private final RelativeEncoder raiseEncoder2 = raiseMotor2.getEncoder();
    private final RelativeEncoder extendEncoder = extendMotor.getEncoder();

    private boolean isCone = false;//Change if initial mode is different

    private final PIDController armExtendPID = new PIDController(0.27891030029999945400000000000, 0, 0);
    private final PIDController armRaisePID1 = new PIDController(0.028600000000000, 0.005, 0.008);
    private final PIDController armRaisePID2 = new PIDController(0.028600000000000, 0.005, 0.008);

    private final Spark LED = new Spark(0);

    public Arm(){
        raiseMotor2.setInverted(true);
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

    public void setIdle(){
        System.out.println("Set idle");
        extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        raiseMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPreset(String stage){
        if(isCone){
            LED.set(0.69);
        }
        if(!isCone){
            LED.set(0.91);
        }
        new Thread(() -> {
            extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get(stage)));
            System.out.println(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get(stage)));
        }).start();
        new Thread(() -> {
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
        new Thread(() -> {
            raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), OperatorConstants.armRaisePresets.get(stage)));
        }).start();
    }

    public void manualArm2(double extend, double raise){
        //raise = raise < 0 ? raise : 1.5*raise;
        extendMotor.set(armExtendPID.calculate(getExtendPosition(), getExtendPosition() + 1.5*extend));
        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), getRaise1Position()));
        raiseMotor2.set(armRaisePID2.calculate(getRaise2Position(), getRaise2Position()));
    }

    public void manualArm(double extend, double raise) {
        extendMotor.set(extend);
        raiseMotor1.set(raise/10);
        raiseMotor2.set(raise/10);
    }
}
