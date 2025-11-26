package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    // Drivetrain Constants
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
    

    // CAN IDs
    public static final int lfDriveMotorID = 1;
    public static final int lbDriveMotorID = 2;
    public static final int rfDriveMotorID = 4;
    public static final int rbDriveMotorID = 3;


}
