package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spitter extends SubsystemBase{
    private SparkMax outtakeMotor;
    private SparkMax outtakeMotorWS;

    public Spitter(){
        
        outtakeMotor = new SparkMax(
            Constants.Swerve.Outtake.outtakeMotorID,
            MotorType.kBrushed
        );
        SparkMaxConfig outtakeMotorConfig = new SparkMaxConfig();

        outtakeMotorWS = new SparkMax(
            Constants.Swerve.Outtake.outtakeMotorWSID,
            MotorType.kBrushed
        );
        SparkMaxConfig outtakeMotorConfigWS = new SparkMaxConfig();
        
    }

    public void OuttakeOut(){
        outtakeMotorWS.set(-.5);
        outtakeMotor.set(-.5);
    }
    public void OuttakeIn(){
        outtakeMotor.set(.5);
        outtakeMotorWS.set(.5);
    }
    public void OuttakeStopManual(){
        outtakeMotor.stopMotor();
        outtakeMotorWS.stopMotor();
    }
    public Command manualOuttakeControl(BooleanSupplier up, BooleanSupplier down){
        return run(()->{
            if (up.getAsBoolean()){
               OuttakeIn();
            }
            else if (down.getAsBoolean()){
                OuttakeOut();
            }
            else{
                OuttakeStopManual();
            }
        });
    
    }

    public void periodic(){

    }
}