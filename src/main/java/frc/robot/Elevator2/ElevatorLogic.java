package frc.robot.Elevator2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;

public class ElevatorLogic extends SubsystemBase{
    
    SmartMotorController elevMotor;

    public ElevatorLogic(ElevatorIO2 io){
        elevMotor = io.getMotor();
    }

    

}
