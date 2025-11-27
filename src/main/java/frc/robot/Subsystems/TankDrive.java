package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.DriveIOInputsAutoLogged;
//import frc.robot.IO.DriveIOInputsAutoLogged;
import frc.robot.IO.TankDriveIO;
import frc.robot.IO.TankDriveIO.DriveIOInputs;

public class TankDrive extends SubsystemBase {
    private TankDriveIO io;
    private DriveIOInputsAutoLogged inputs;

    public TankDrive(TankDriveIO io) {
        inputs = new DriveIOInputsAutoLogged();
        this.io = io;
    }

    public Command getVoltageCmd(Supplier<Voltage> leftSupplier, Supplier<Voltage> rightSupplier) {
        return this.runEnd(
                () -> io.setVoltage(leftSupplier.get(), rightSupplier.get()),
                () -> io.setVoltage(Volts.zero(), Volts.zero()));
    }

    public Command getRateCmd(Supplier<LinearVelocity> leftSupplier, Supplier<LinearVelocity> rightSupplier){
        return this.runEnd(
                () -> io.setLinRate(leftSupplier.get(), rightSupplier.get()),
                () -> io.setVoltage(Volts.zero(), Volts.zero()));
    }

    public Command getPosCmd(Supplier<Distance> leftSupplier, Supplier<Distance> rightSupplier){
        return this.runEnd(
                () -> io.setLinPos(leftSupplier.get(), rightSupplier.get()),
                () -> io.setVoltage(Volts.zero(), Volts.zero()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

}
