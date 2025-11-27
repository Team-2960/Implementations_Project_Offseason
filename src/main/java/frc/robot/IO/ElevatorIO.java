package frc.robot.IO;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    
    @AutoLog
    public class ElevatorIOInputs {
        Voltage elevVoltage = Volts.zero();
        Current elevCurrent = Amps.zero();
        LinearVelocity elevLinVelocity = MetersPerSecond.zero();
        AngularVelocity elevAngVelocity = RotationsPerSecond.zero();
        Distance elevDistance = Meters.zero();
        Angle elevAngle = Radians.zero();
    }
    
    public void updateInputs(ElevatorIOInputs inputs);

    public void setVoltage(Voltage volts);

    public default void setLinRate(LinearVelocity rate) {
    }

    public default void setAngRate(AngularVelocity rate) {
    }

    public default void setLinPos(Distance distance) {
    }

    public default void setAngPos(Angle angle) {}

    public default void getVoltageCmd(Supplier<Voltage> volts) {}

    public default void getLinRateCmd(Supplier<LinearVelocity> rate) {}

    public default void getAngRateCmd(Supplier<AngularVelocity> rate) {}

    public default void getLinPosCmd(Supplier<Distance> distance) {}

    public default void getAngPosCmd(Supplier<Distance> angle) {}


}
