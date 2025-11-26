package frc.robot.IO;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

public class TankDriveIOSpark implements TankDriveIO {

    private SparkMax lfMotor;
    private SparkMax lbMotor;
    private SparkMax rfMotor;
    private SparkMax rbMotor;

    private SmartMotorController leftMotor;
    private SmartMotorController rightMotor;

    /**
     * 
     * Will automatically add follower configurations, so need to do them before
     * adding them into the contructor parameters.
     * Use super(...) when extending this class
     * 
     * @param lfMotorID   left front motor CAN ID (left leader)
     * @param lbMotorID   left back motor CAN ID (left follower)
     * @param rfMotorID   right front motor CAN ID (left leader)
     * @param rbMotorID   right back motor CAN ID (left follower)
     * @param leftConfig  left motor configuration
     * @param rightConfig right motor configuration
     */
    public TankDriveIOSpark(int lfMotorID, int lbMotorID, int rfMotorID, int rbMotorID,
            SmartMotorControllerConfig leftConfig, SmartMotorControllerConfig rightConfig) {
        lfMotor = new SparkMax(lfMotorID, MotorType.kBrushless);
        lbMotor = new SparkMax(lbMotorID, MotorType.kBrushless);
        rfMotor = new SparkMax(rfMotorID, MotorType.kBrushless);
        rbMotor = new SparkMax(rbMotorID, MotorType.kBrushless);

        // lfMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);
        // rfMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);

        // leftConfig.follow(lfMotorID);
        // rightConfig.follow(rfMotorID);

        // lbMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);
        // rbMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);

        leftConfig.withFollowers(Pair.of(lbMotor, false));
        leftMotor.applyConfig(leftConfig);

        rightConfig.withFollowers(Pair.of(rbMotor, false));
        rightMotor.applyConfig(rightConfig);

        getSysId(Volts.of(12), Volts.of(2).per(Second), null);
    }

    @Override
    public void setVoltage(Voltage leftVolts, Voltage rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(rightVolts);
    }

    @Override
    public void setLinRate(LinearVelocity leftRate, LinearVelocity rightRate) {
        leftMotor.setVelocity(leftRate);
        rightMotor.setVelocity(rightRate);
    }

    @Override
    public void setLinPos(Distance leftDist, Distance rightDist) {
        leftMotor.setPosition(leftDist);
        rightMotor.setPosition(rightDist);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftVoltage = leftMotor.getVoltage();
        inputs.leftCurrent = leftMotor.getSupplyCurrent().get();
        inputs.leftLinVelocity = leftMotor.getMeasurementVelocity();
        inputs.leftAngVelocity = leftMotor.getMechanismVelocity();
        inputs.leftDistance = leftMotor.getMeasurementPosition();
        inputs.leftAngle = leftMotor.getMechanismPosition();

        inputs.rightVoltage = rightMotor.getVoltage();
        inputs.rightCurrent = rightMotor.getSupplyCurrent().get();
        inputs.rightLinVelocity = rightMotor.getMeasurementVelocity();
        inputs.rightAngVelocity = rightMotor.getMechanismVelocity();
        inputs.rightDistance = rightMotor.getMeasurementPosition();
        inputs.rightAngle = rightMotor.getMechanismPosition();
    }

    @Override
    public SysIdRoutine getSysId(Voltage maxVoltage, Velocity<VoltageUnit> stepVoltage, Time testDuration) {
        return leftMotor.sysId(maxVoltage, stepVoltage, testDuration);
    }
}
