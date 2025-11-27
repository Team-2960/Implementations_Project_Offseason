package frc.robot.IO;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSparkIO implements ElevatorIO{

    private SparkMax elevSpark;
    private SmartMotorController elevMotor;
    private static ElevatorSparkIO elevatorSparkIO = null;
    
    public ElevatorSparkIO(SparkBase sparkController, SmartMotorControllerConfig config, DCMotor DCMotorType){
        elevMotor = new SparkWrapper(sparkController, DCMotorType, config);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevVoltage = elevMotor.getVoltage();
    }

    @Override
    public void setVoltage(Voltage volts) {
        elevMotor.setVoltage(volts);
    }


}
