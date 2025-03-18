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
import java.util.concurrent.TimeUnit;


public class Spitter extends SubsystemBase {
    private SparkMax outtakeMotor;
    private SparkMax outtakeMotorWS;
    private LaserCan laser1;
    private LaserCan laser2;
    private int l1count = 0;
    private int l2count = 0;
    private boolean prevlaser1val;
    private boolean prevlaser2val;
    private boolean first = true; // True is intake mode, False is scoring mode
    private boolean coralPosessed = false;
    public Spitter() {
        laser1 = new LaserCan(18);     
        laser2 = new LaserCan(17);    
        prevlaser1val = false;
        prevlaser2val = false; 
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
        if (coralPosessed){

            outtakeMotorWS.set(.5);
            outtakeMotor.set(-.5);
        }
        else if (!coralPosessed){
            if (laser2.getMeasurement().distance_mm < 200){
                outtakeMotorWS.set(.35);
                outtakeMotor.set(-.35);
            }
            else{
                outtakeMotorWS.set(.45);
                outtakeMotor.set(-.45);
            }
        }
            
    }

    public void OuttakeIn() {
        outtakeMotor.set(-.6);
        outtakeMotorWS.set(.6);
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
        if (laser1.getMeasurement() != null){
            return laser1.getMeasurement().distance_mm < Constants.Outtake.kLaserDistCali1;
        }
        else{
            return false;
        }
    }
    public boolean checklaser2(){
        if (laser2.getMeasurement() != null){
            return laser2.getMeasurement().distance_mm < Constants.Outtake.kLaserDistCali2;
        }
        else{
            return false;
        }
        // return laser2.getMeasurement().distance_mm < Constants.Outtake.kLaserDistCali2;
    }
    public boolean checkCoral(){
        if (first){
            return l1count == 2 && l2count == 1;
        }
        else{
            return l2count == 1;
        }
    }
    
    public void periodic() {
        // if (laser1.getMeasurement() != null ){
        //     if (laser1.getMeasurement().distance_mm < 50){
        //         System.out.println("triggered1");
        //     }
        //     else{
        //         System.out.println("not trigger1");
        //     }
        // }
        // if (laser2.getMeasurement() != null){
        //     if (laser2.getMeasurement().distance_mm < 100){
        //         System.out.println("triggered2");
        //     }
        //     else{
        //         System.out.println("not trigger2");
        //     }

        // }
        // else{
        //     System.out.println("null");
        // // }
        // System.out.println(l1count);
        // System.out.println(l2count);
        // if (checklaser1() != prevlaser1val){
        //     l1count += 1;
        //     prevlaser1val = checklaser1();
        // }
        // if (checklaser2() != prevlaser2val){
        //     l2count += 1;
        //     prevlaser2val = checklaser2();
        // }
        // if (checklaser1() != prevlaser1val){
        //     l1count +=1;
        //     prevlaser1val = checklaser1();
        // }
        // if (checklaser2() != prevlaser2val){
        //     l2count +=1;
        //     prevlaser2val = checklaser2();
        // }
        // System.out.println(checklaser2() + " " + l1count);
        // System.out.println(checklaser1() + " " + l2count);
        if (coralPosessed != true){

            if (laser2.getMeasurement().distance_mm > 200 && laser1.getMeasurement().distance_mm < 100){
                OuttakeStopManual();
                first = true;
                coralPosessed = true;
                
                
                
            }
        }
        else{
            // Ejection mode
            System.out.println("ejaculrion mode: " + laser1.getMeasurement().distance_mm);
            if (laser1.getMeasurement().distance_mm > 100 ){
                System.out.println("not posessed anymore");
                OuttakeStopManual();
                coralPosessed = false;
            }
        }
        // If mode is outtake mode
        // if (mode){
        //     // And coral is detected as possessed
        //     boolean coralPossession = checkCoral();
        //     // Switch to scoring mode
        //     if (coralPossession){
        //         System.out.println("coral possessed");        
        //         OuttakeStopManual();
        //         l1count = 0;
        //         l2count = 0;
        //         mode = false; 
        //     }
        // }
        // else{
        //     // If mode is scoring mode
        //     // And coral is detected as scored
        //     boolean coralScored = checkCoral();
        //     // Switch to intake mode
        //     if (coralScored){
        //         OuttakeStopManual();
        //         l1count = 0;
        //         l2count = 0;
        //         mode = true; 
        //     }   
        // }

        SmartDashboard.putBoolean("Coral Possessed?", coralPosessed);
    }
}