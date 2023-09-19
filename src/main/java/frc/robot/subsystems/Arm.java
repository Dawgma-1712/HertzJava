package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final PIDController armPID;

    public Arm(){
        this.raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
        this.raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
        this.extendMotor = new CANSparkMax(15, MotorType.kBrushless);

        raiseEncoder1 = raiseMotor1.getEncoder();
        raiseEncoder2 = raiseMotor2.getEncoder();
        extendEncoder = extendMotor.getEncoder();

        armPID = new PIDController(0.01, 0, 0);
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
    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
        extendMotor.stopMotor();
    }

    public void coneLow(){
        extendMotor.set(armPID.calculate(getExtendPosition(), 0.01));
    }
}