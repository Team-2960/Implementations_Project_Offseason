package frc.robot.IO;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface TankDriveIO {

    @AutoLog
    public class DriveIOInputs {
        Voltage leftVoltage = Volts.zero();
        Current leftCurrent = Amps.zero();
        LinearVelocity leftLinVelocity = MetersPerSecond.zero();
        AngularVelocity leftAngVelocity = RotationsPerSecond.zero();
        Distance leftDistance = Meters.zero();
        Angle leftAngle = Radians.zero();

        Voltage rightVoltage = Volts.zero();
        Current rightCurrent = Amps.zero();
        LinearVelocity rightLinVelocity = MetersPerSecond.zero();
        AngularVelocity rightAngVelocity = RotationsPerSecond.zero();
        Distance rightDistance = Meters.zero();
        Angle rightAngle = Radians.zero();
    }

    public void updateInputs(DriveIOInputs inputs);

    public void setVoltage(Voltage leftVolts, Voltage rightVolts);

    public default void setLinRate(LinearVelocity leftRate, LinearVelocity rightRate) {
    }

    public default void setAngRate(AngularVelocity leftRate, AngularVelocity rightRate) {
    }

    public default void setLinPos(Distance leftDist, Distance rightDist) {
    }

    public default void setAngPos(Distance leftAng, Distance rightAng) {
    }

    public default SysIdRoutine getSysId(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration) {
        return null;
    }

}
