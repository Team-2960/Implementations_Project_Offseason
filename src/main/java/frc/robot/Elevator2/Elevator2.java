package frc.robot.Elevator2;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class Elevator2 extends ElevatorLogic{

    private static Elevator2 elevator2 = null;
    
    public Elevator2(){
        super(new ElevatorSparkIO2(new SparkMax(0, MotorType.kBrushless), DCMotor.getNEO(1), new SmartMotorControllerConfig(getInstance())));
    }

    public static Elevator2 getInstance(){
        if ( elevator2 == null){
            elevator2 = new Elevator2();
        } 

        return elevator2;
    }


}
