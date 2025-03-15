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



public class Elevator extends SubsystemBase{
    
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotorWS;
    private SparkMax outtakeMotor;
    private SparkMax outtakeMotorWS;
    private PWM trapServo;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;
    private Encoder elevatorMotorEncoder;
    // private RelativeEncoder elevatorMotorEncoder;
    private SparkClosedLoopController elevatorMotorPID;
    private SparkClosedLoopController elevatorMotorWSPID;

    public Elevator(){
        trapServo = new PWM(Constants.Swerve.trapServoID);
        limitSwitch1 = new DigitalInput(Constants.Swerve.limitSwitch1ID);
        limitSwitch2 = new DigitalInput(Constants.Swerve.limitSwitch2ID);        
        //positive power percentage is elevator up
        elevatorMotor = new SparkMax(
            Constants.Swerve.elevator.elevatorShaft.shaftMotorID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.idleMode(IdleMode.kBrake).inverted(false);
        elevatorMotorConfig.closedLoop.pid(Constants.Swerve.elevator.elevatorShaft.kP, Constants.Swerve.elevator.elevatorShaft.kI, Constants.Swerve.elevator.elevatorShaft.kD);
        //TODO elevatorMotorConfig.encoder.velocityConversionFactor(())
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //positive power percentage is elevator down
        elevatorMotorWS = new SparkMax(
            Constants.Swerve.elevator.elevatorShaft.shaftMotorWSID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorWSConfig = new SparkMaxConfig();
        elevatorMotorWSConfig.idleMode(IdleMode.kBrake).inverted(true);
        elevatorMotorWSConfig.closedLoop.pid(Constants.Swerve.elevator.elevatorShaft.kP, Constants.Swerve.elevator.elevatorShaft.kI, Constants.Swerve.elevator.elevatorShaft.kD);
        //TODO elevatorMotorWSConfig.encoder.velocityConversionFactor(())
        elevatorMotorWS.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        outtakeMotor = new SparkMax(
            Constants.Swerve.elevator.elevatorOuttake.outtakeMotorID,
            MotorType.kBrushed
        );
        SparkMaxConfig outtakeMotorConfig = new SparkMaxConfig();

        outtakeMotorWS = new SparkMax(
            Constants.Swerve.elevator.elevatorOuttake.outtakeMotorWSID,
            MotorType.kBrushed
        );
        SparkMaxConfig outtakeMotorConfigWS = new SparkMaxConfig();
        elevatorMotorEncoder = new Encoder(Constants.Swerve.throBoreRelativeChannel1,Constants.Swerve.throBoreRelativeChannel2);//elevatorMotor.getEncoder();

        // Set the position to zero
        elevatorMotorEncoder.reset();
    }
    public void reset(){
        elevatorMotorEncoder.reset();
    }
    
   
    public Command Stop(){
        return run(()->{
            elevatorMotor.set(0);
            elevatorMotorWS.set(0);
        });
    }

    public Rotation2d getPos(){
        Rotation2d posElevatorMotor = Rotation2d.fromDegrees(elevatorMotorEncoder.get()); 
        return posElevatorMotor;
    }
    public Rotation2d getErrors(Rotation2d goal){
        Rotation2d currPos =getPos();
        double error = currPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }
    // Run PID
    public void GoTo(Rotation2d goal){
        elevatorMotorPID.setReference(
            goal.getDegrees(), 
            ControlType.kPosition, ClosedLoopSlot.kSlot0
        );
        
    }
    // Convert PID method into a runable command
    // if button is pressed, go down to goal, until within setpoint or when button stopped pressing, when button at setpoint, start intake and go until 
    public Command moveTo(Rotation2d goal, boolean intake){
        
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Goal", goal.getDegrees());
           
        }).andThen(run(
            () -> GoTo(goal)
        ).withTimeout(1)/*until(
            ()->(
                Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance 
                && Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance
            )
        ).andThen((intake)? In(): Stop())*/
        );
    }


 // Manual Controls:
 public void ShaftUp(){
    elevatorMotorWS.set(.5);
    elevatorMotor.set(.5);
}
public void ShaftDown(){
    elevatorMotor.set(.5);
    elevatorMotorWS.set(.5);
}
public void ShaftStopManual(){
    elevatorMotor.stopMotor();
    elevatorMotorWS.stopMotor();
}
public Command manualShaftControl(BooleanSupplier up, BooleanSupplier down){
    return run(()->{
        if (up.getAsBoolean()){
           ShaftUp(); 
           System.out.print("going up");
        }
        else if (down.getAsBoolean()){
            ShaftDown();
        }
        else{
            ShaftStopManual();
        }
    });

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
    if (limitSwitch1.get() && limitSwitch2.get()){
        reset();
    }

    SmartDashboard.putNumber("Shaft pos", getPos().getDegrees());
}
}