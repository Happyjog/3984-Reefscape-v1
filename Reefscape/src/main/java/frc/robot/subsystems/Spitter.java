package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spitter extends SubsystemBase {
    private SparkMax outtakeMotor;
    private SparkMax outtakeMotorWS;
    private LaserCan laser1;
    private LaserCan laser2;
    private int l1count = 0;
    private int l2count = 0;
    private boolean prevlaser1val;
    private boolean prevlaser2val;
    private boolean mode; // True is intake mode, False is scoring mode
    public Spitter() {
        laser1 = new LaserCan(0);     
        laser2 = new LaserCan(0);    
        prevlaser1val = checklaser1();
        prevlaser2val = checklaser2(); 
        outtakeMotor = new SparkMax(
                Constants.Outtake.outtakeMotorID,
                MotorType.kBrushed);
        SparkMaxConfig outtakeMotorConfig = new SparkMaxConfig();
        outtakeMotor.configure(outtakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        outtakeMotorWS = new SparkMax(
                Constants.Outtake.outtakeMotorWSID,
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

    public Command manualOuttakeControl(BooleanSupplier in, BooleanSupplier out) {
        return run(() -> {
            if (in.getAsBoolean()) {
                OuttakeIn();
            } else if (out.getAsBoolean()) {
                OuttakeOut();
            } else {
                OuttakeStopManual();
            }
        });

    }

    public boolean checklaser1(){
        return laser1.getMeasurement().distance_mm < Constants.Outtake.kLaserDistCali1;
    }
    public boolean checklaser2(){
        return laser2.getMeasurement().distance_mm < Constants.Outtake.kLaserDistCali2;
    }
    public boolean checkCoral(){
        if (mode){
            return l1count == 2 && l2count == 1;
        }
        else{
            return l2count == 1;
        }
    }
    
    public void periodic() {
        if (checklaser1() != prevlaser1val){
            l1count += 1;
            prevlaser1val = checklaser1();
        }
        if (checklaser2() != prevlaser2val){
            l2count += 1;
            prevlaser2val = checklaser2();
        }
        // If mode is outtake mode
        if (mode){
            // And coral is detected as possessed
            boolean coralPossession = checkCoral();
            // Switch to scoring mode
            if (coralPossession){
                OuttakeStopManual();
                l1count = 0;
                l2count = 0;
                mode = false; 
            }
        }
        else{
            // If mode is scoring mode
            // And coral is detected as scored
            boolean coralScored = checkCoral();
            // Switch to intake mode
            if (coralScored){
                OuttakeStopManual();
                l1count = 0;
                l2count = 0;
                mode = true; 
            }   
        }

        SmartDashboard.putBoolean("Coral Possessed?", mode);
    }
}