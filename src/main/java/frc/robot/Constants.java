package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    // Drivetrain Constants
    public static final Current maxDriveCurrent = Amps.of(60);
    public static final Distance wheelDiameter = Inches.of(3);
    public static final double driveRatio = 1.0/8.45;
    public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(3);
    public static final LinearAcceleration maxDriveAccel = MetersPerSecondPerSecond.of(0.6);
    public static final Distance driveRampDownDist = Meters.of(0.2);
    public static final Distance trackWidth = Inches.of(21.9375);
    public static final ProfiledPIDController tankDrivePID = 
        new ProfiledPIDController(0.20798, 0, 0, 
            new Constraints(
                maxDriveSpeed.in(MetersPerSecond), 
                maxDriveAccel.in(MetersPerSecondPerSecond)
            )
        );
    public static final SimpleMotorFeedforward tankDriveFF = new SimpleMotorFeedforward(0.17074, 1.9301, 0.55485);

    public static final double driveVolt = 12;
    

    //Elevator Constants
    public static final AngularVelocity maxElevatorSpeed = AngularVelocity.ofBaseUnits(12, RotationsPerSecond);
    //TODO Change Accel value
    public static final AngularAcceleration maxElevatorAccel = RotationsPerSecondPerSecond.of(3);
    public static final double elevatorScale = 1;
    public static final Angle elevatorRampDownDist = Angle.ofBaseUnits(0.5, Rotations);
    public static final Angle elevTopLim = Rotations.of(17.22);
    public static final Angle elevBotLim = Rotations.of(0.2);
    public static final ProfiledPIDController elevPID = new ProfiledPIDController(elevatorScale, driveVolt, driveRatio, new Constraints(maxElevatorSpeed.in(RotationsPerSecond), maxElevatorAccel.in(RotationsPerSecondPerSecond)));
    //TODO Change to Elevator Feed Forward
    public static final SimpleMotorFeedforward elevFF = new SimpleMotorFeedforward(0.15215, 0.66776, 0.031198);
    // CAN IDs
    public static final int lfDriveMotorID = 1;
    public static final int lbDriveMotorID = 2;
    public static final int rfDriveMotorID = 4;
    public static final int rbDriveMotorID = 3;

    public static final int elevMotorID = 7;


}
