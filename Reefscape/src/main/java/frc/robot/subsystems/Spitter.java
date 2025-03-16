package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spitter extends SubsystemBase {
    private SparkMax outtakeMotor;
    private SparkMax outtakeMotorWS;

    public Spitter() {

        outtakeMotor = new SparkMax(
                Constants.Swerve.Outtake.outtakeMotorID,
                MotorType.kBrushed);
        SparkMaxConfig outtakeMotorConfig = new SparkMaxConfig();
        outtakeMotor.configure(outtakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        outtakeMotorWS = new SparkMax(
                Constants.Swerve.Outtake.outtakeMotorWSID,
                MotorType.kBrushed);
        SparkMaxConfig outtakeMotorConfigWS = new SparkMaxConfig();
        outtakeMotorWS.configure(outtakeMotorConfigWS, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void OuttakeOut() {
        outtakeMotorWS.set(-.5);
        outtakeMotor.set(-.5);
    }

    public void OuttakeIn() {
        outtakeMotor.set(.5);
        outtakeMotorWS.set(.5);
    }

    public void OuttakeStopManual() {
        outtakeMotor.stopMotor();
        outtakeMotorWS.stopMotor();
    }

    public Command manualOuttakeControl(BooleanSupplier up, BooleanSupplier down) {
        return run(() -> {
            if (up.getAsBoolean()) {
                OuttakeIn();
            } else if (down.getAsBoolean()) {
                OuttakeOut();
            } else {
                OuttakeStopManual();
            }
        });

    }

    public void periodic() {

    }
}