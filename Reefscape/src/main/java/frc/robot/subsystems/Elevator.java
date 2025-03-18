package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.MathUtils;



public class Elevator extends SubsystemBase{
    
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotorWS;
    
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;
    
    private Encoder elevatorMotorEncoder;
    // private RelativeEncoder elevatorMotorEncoder;
    // private SparkClosedLoopController elevatorMotorPID;
    // private SparkClosedLoopController elevatorMotorWSPID;
    private PIDController elevatorMotorPID;
    private ProfiledPIDController elevatorMotorProfiledPID;
    private ElevatorFeedforward elevatorFeedforward;

    public Elevator(){
        limitSwitch1 = new DigitalInput(Constants.Swerve.limitSwitch1ID);
        limitSwitch2 = new DigitalInput(Constants.Swerve.limitSwitch2ID);   
        
        //positive power percentage is elevator up
        elevatorMotor = new SparkMax(
            Constants.Elevator.elevatorShaft.shaftMotorID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.idleMode(IdleMode.kBrake).inverted(false);
        elevatorMotorConfig.closedLoop.pid(Constants.Elevator.elevatorShaft.kP, Constants.Elevator.elevatorShaft.kI, Constants.Elevator.elevatorShaft.kD);
        elevatorMotorConfig.smartCurrentLimit(40).closedLoopRampRate(Constants.Elevator.elevatorShaft.kElevatorMotorRampRate);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //positive power percentage is elevator down
        elevatorMotorWS = new SparkMax(
            Constants.Elevator.elevatorShaft.shaftMotorWSID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorWSConfig = new SparkMaxConfig();
        elevatorMotorWSConfig.idleMode(IdleMode.kBrake).inverted(false);
        elevatorMotorWSConfig.closedLoop.pid(Constants.Elevator.elevatorShaft.kP, Constants.Elevator.elevatorShaft.kI, Constants.Elevator.elevatorShaft.kD);
        elevatorMotorConfig.smartCurrentLimit(40).closedLoopRampRate(Constants.Elevator.elevatorShaft.kElevatorMotorRampRate);
        elevatorMotorWS.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorEncoder = new Encoder(Constants.Swerve.throBoreRelativeChannel1, Constants.Swerve.throBoreRelativeChannel2);
        elevatorMotorPID = new PIDController(
            Constants.Elevator.elevatorShaft.kP, 
            Constants.Elevator.elevatorShaft.kI, 
            Constants.Elevator.elevatorShaft.kD);
        elevatorMotorProfiledPID  = new ProfiledPIDController(
            Constants.Elevator.elevatorShaft.kP, 
            Constants.Elevator.elevatorShaft.kI, 
            Constants.Elevator.elevatorShaft.kD, 
            Constants.Elevator.elevatorShaft.kElevatorConstraints);
        elevatorFeedforward = new ElevatorFeedforward(
            Constants.Elevator.elevatorShaft.kS, 
            Constants.Elevator.elevatorShaft.kG, 
            Constants.Elevator.elevatorShaft.kV, 
            Constants.Elevator.elevatorShaft.kA);
        
        // Set the position to zero
        elevatorMotorEncoder.reset();
    }


    // // Run PID
    // public void GoTo(Rotation2d goal){
    //     elevatorMotorPID.setReference(
    //         goal.getDegrees(), 
    //         ControlType.kPosition, ClosedLoopSlot.kSlot0
    //     );
        
    // }
    // // Convert PID method into a runable command
    // // if button is pressed, go down to goal, until within setpoint or when button stopped pressing, when button at setpoint, start intake and go until 
    // public Command moveTo(Rotation2d goal, boolean intake){
        
    //     //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
    //     return runOnce(()->{
    //         SmartDashboard.putNumber("Goal", goal.getDegrees());
           
    //     }).andThen(run(
    //         () -> GoTo(goal)
    //     ).withTimeout(1)/*until(
    //         ()->(
    //             Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance 
    //             && Math.abs(getErrors(goal).getDegrees()) < Constants.Swerve.intake.tolerance
    //         )
    //     ).andThen((intake)? In(): Stop())*/
    //     );
    // }


    // Unprofiled PID Control
    public void reachGoalNoProfile(double goal){
        double voltageOutput = MathUtils.clamp(elevatorMotorPID.calculate(getPos().getDegrees(), goal), 7, -7);
        elevatorMotor.setVoltage(-voltageOutput);
        elevatorMotorWS.setVoltage(voltageOutput);
    }
    // Profiled PID Control + ff
    public void reachGoal(double goal){
        double voltageOutput = MathUtils.clamp(elevatorFeedforward.calculateWithVelocities(getVelocity(), elevatorMotorProfiledPID.getSetpoint().velocity)
            + elevatorMotorProfiledPID.calculate(getPos().getDegrees(), goal), 7, -7); 
        elevatorMotor.setVoltage(-voltageOutput);
        elevatorMotorWS.setVoltage(voltageOutput);
    }
    //PID Runner
    public Command setHeight(double goal){
        System.out.println("ran");
        System.out.println(Math.abs(getPos().getDegrees() - goal));
        return run(()->{System.out.println(Math.abs(getPos().getDegrees() - goal));}).until(()->Math.abs(getPos().getDegrees() - goal) < 0);
    }


    // Manual Controls:
    public void ShaftUp(){
        elevatorMotorWS.set(-.05);
        elevatorMotor.set(.05);
    }
    public void ShaftDown(){
        elevatorMotor.set(-.05);
        elevatorMotorWS.set(.05);
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
    // Get and Reset methods:
    public Rotation2d getPos(){
        Rotation2d posElevatorMotor = Rotation2d.fromDegrees(elevatorMotorEncoder.get()); 
        return posElevatorMotor;
    }
    public void resetPos(){
        elevatorMotorEncoder.reset();
    }
    public double getVelocity(){
        return elevatorMotorEncoder.getRate();
    }
    public Rotation2d getErrors(Rotation2d goal){
        Rotation2d currPos =getPos();
        double error = currPos.getDegrees() - goal.getDegrees();
        return Rotation2d.fromDegrees(error);
    }

    // Periodic
    public void periodic(){
        // If elevator has reached bottom, reset to 0 to avoid drift and stuff.
        if (!limitSwitch1.get() && !limitSwitch2.get()){
            resetPos();
        }
        SmartDashboard.putNumber("Shaft pos", getPos().getDegrees());
    }
}