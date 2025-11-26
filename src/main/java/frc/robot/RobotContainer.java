// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IO.TankDriveIOSpark;
import frc.robot.Subsystems.TankDrive;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class RobotContainer {

  private TankDriveIOSpark tankDriveIO;
  private TankDrive tankDrive;
  private SmartMotorControllerConfig tankDriveConfig;

  private CommandXboxController driverCtrl;

  private final MutVoltage leftCtrlVolt = Volts.mutable(0);
  private final MutVoltage rightCtrlVolt = Volts.mutable(0);

  public RobotContainer() {

    tankDriveConfig = new SmartMotorControllerConfig(tankDrive)
        .withGearing(0)
        .withClosedLoopController(Constants.tankDrivePID)
        .withFeedforward(Constants.tankDriveFF)
        .withGearing(Constants.driveRatio)
        .withMechanismCircumference(Constants.trackWidth)
        .withStartingPosition(Meter.zero());

    tankDriveIO = new TankDriveIOSpark(0, 1, 2, 3,
        tankDriveConfig,
        tankDriveConfig.withMotorInverted(true));

    tankDrive = new TankDrive(tankDriveIO);

    configureBindings();
  }

  private void configureBindings() {
    driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1).onTrue(
        tankDrive.getVoltageCmd(
            () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * Constants.driveVolt,
                Volts),
            () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * Constants.driveVolt,
                Volts)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
