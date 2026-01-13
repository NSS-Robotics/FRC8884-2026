package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Feeder/Indexer => drumroll, please... => Finndexer!!!
public class Finndexer extends SubsystemBase {
    private final TalonFX feederMotor = new TalonFX(Constants.CanIDs.feederMotorID);
    // private final TalonFX indexerMotor = new TalonFX(Constants.CanIDs.mainIndexerMotorID);
    private final TalonFX followerIndexerMotor = new TalonFX(Constants.CanIDs.followerIndexerMotorID);

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public Finndexer() {
        Slot0Configs feederPIDConfig = new Slot0Configs();
        feederPIDConfig.kP = Constants.Finndexer.kP;
        feederPIDConfig.kI = Constants.Finndexer.kI;
        feederPIDConfig.kD = Constants.Finndexer.kD;
        feederPIDConfig.kS = Constants.Finndexer.kS;
        feederPIDConfig.kV = Constants.Finndexer.kV;
        feederPIDConfig.kA = Constants.Finndexer.kA;
        feederMotor.getConfigurator().apply(feederPIDConfig);
        // indexerMotor.getConfigurator().apply(feederPIDConfig);

        feederMotor.setNeutralMode(NeutralModeValue.Coast);
        // indexerMotor.setNeutralMode(NeutralModeValue.Coast);

        // followerIndexerMotor.setControl(new Follower(indexerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public double getFeederVelocity() {
        return feederMotor.getVelocity().getValueAsDouble();
    }

    public double getIndexerVelocity() {
        // return indexerMotor.getVelocity().getValueAsDouble();
        return 0;
    }

    public void setVelocity(double velocityRPM) {
        feederMotor.setControl(velocityVoltage.withVelocity(velocityRPM / 60));
        // indexerMotor.setControl(velocityVoltage.withVelocity(velocityRPM / 60));
    }

    public void makeItSpin(double power) {
        feederMotor.set(power);
    }

    public void stop() {
        feederMotor.setControl(new VoltageOut(0));
        // indexerMotor.setControl(new VoltageOut(0));
    }
}
