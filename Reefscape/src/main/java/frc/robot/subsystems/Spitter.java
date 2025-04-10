package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public String[] states = {"Coral Intake", "Coral Posessed", "Coral Scoring"};
    private boolean low = false;
    public String curr_state = states[0];
    private boolean first = false; // True is intake mode, False is scoring mode
    private boolean inter = false; // True is intake mode, False is scoring mode

    private boolean coralPosessed = false;
    
    public Spitter() {
        laser1 = new LaserCan(17);     
        laser2 = new LaserCan(18);    
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
    public void runOutake(){
        if (curr_state.equals(states[0])){
            if (inter){
                outtakeMotorWS.set(.3);
                outtakeMotor.set(-.3);
            }
            outtakeMotorWS.set(.45);
            outtakeMotor.set(-.45);

        }
        
        else if(curr_state.equals(states[1])){
            curr_state = states[2];
            if (low){
                outtakeMotorWS.set(.8);
                outtakeMotor.set(-.8);
            }
            else{
                outtakeMotorWS.set(.5);
                outtakeMotor.set(-.5);
            }
            
        }
        
    }
    public void OuttakeOut() {

        if (coralPosessed){
            outtakeMotorWS.set(.1);
            outtakeMotor.set(-.1);
        }
        else if (!coralPosessed){
            if (laser2.getMeasurement().distance_mm < 200){
                outtakeMotorWS.set(0.1);//.35);
                outtakeMotor.set(-0.1);//.35);
            }
            else{
                outtakeMotorWS.set(0.1);//.45);
                outtakeMotor.set(-0.1);//-.45);
            }
        }
            
    }

    public void OuttakeIn() {
        outtakeMotor.set(.1);
        outtakeMotorWS.set(-.1);
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

    public boolean checklaserFront(){
        if(!(laser1.getMeasurement() == null) && (laser1.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)){
            if (laser1.getMeasurement().distance_mm < 100 ){
                return true;
            }
            return false;
        }else{
            return false;
        }
    }
    public boolean checklaserBack(){
        if(!(laser2.getMeasurement() == null) && (laser2.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)){
            if (laser2.getMeasurement().distance_mm < 100 ){
                return true;
            }
            return false;
        }else{
            System.out.println("back_null");

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

    // state [0] = intake, state [1] = posessed, state [2] = scoring
    public void periodic() {
        String level = NetworkTableInstance.getDefault().getTable("ButtonBoard").getEntry("cumber").getString("");
        if (level.equals("L2") || level.equals("L1") || level.equals("R2") || level.equals("R1")){
            low = true;
        }
        else if (level.equals("L3") || level.equals("R3") ){
            low = false;

            
        }
        else{
            low = true;
        }
        // If current state is intake mode, check for coral posession
        if (laser2.getMeasurement() != null){
            if (laser1.getMeasurement() != null){
                if (curr_state.equals(states[0])){
                    // FIRST, if first laser triggered
                    if (checklaserBack()){
                        first = true;
                    }
                    // THEN, check when first laser is no longer triggered, second laser is trigered . Now it is posessed
                    if (first && !checklaserBack() && checklaserFront()){
                        first = false;
                        inter = false;
                        OuttakeStopManual();
                        curr_state = states[1];
                    }
                    if (checklaserBack() && checklaserFront()){
                        inter = true;
                    }
                }
                if (curr_state.equals(states[1])){
                    OuttakeStopManual();
                }
                if (curr_state.equals(states[2])){
                    // First make sure laser 2 is still triggered, coral is still posessed, if not, go back to intake mode
                    if (!checklaserFront()){
                        OuttakeStopManual();
                        curr_state = states[0];
                    }
                }
            }
        }

        if (curr_state.equals(states[1])){
            SmartDashboard.putBoolean("Loaded", true);
        }
        else{
            SmartDashboard.putBoolean("Loaded", false);
        }
        
        SmartDashboard.putBoolean("lasercanBack", checklaserBack());
        SmartDashboard.putBoolean("lasercanFront", checklaserFront());

        SmartDashboard.putString("Intake state", curr_state);
    }
}