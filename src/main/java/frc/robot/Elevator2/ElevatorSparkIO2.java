package frc.robot.Elevator2;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSparkIO2 implements ElevatorIO2{
    
    private SmartMotorController elevMotor;

    public ElevatorSparkIO2(SparkBase elevSpark, DCMotor dcMotor, SmartMotorControllerConfig motorConfig){
        elevMotor = new SparkWrapper(elevSpark, dcMotor, motorConfig);
    }

    @Override
    public SmartMotorController getMotor() {
        return elevMotor;
    }

}
