package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase{
    
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotorWS;
    private RelativeEncoder elevatorMotorEncoder;
    private SparkClosedLoopController elevatorMotorPID;
    private SparkClosedLoopController elevatorMotorWSPID;

    public Elevator(){

        //positive power percentage is elevator up
        elevatorMotor = new SparkMax(
            Constants.Swerve.elevator.rotMotorID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.idleMode(IdleMode.kCoast).inverted(false);
        elevatorMotorConfig.closedLoop.pid(Constants.Swerve.elevator.kP, Constants.Swerve.elevator.kI, Constants.Swerve.elevator.kD);
        //TODO elevatorMotorConfig.encoder.velocityConversionFactor(())
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //positive power percentage is elevator down
        elevatorMotorWS = new SparkMax(
            Constants.Swerve.elevator.rotMotorID,
            MotorType.kBrushless
        );
        SparkMaxConfig elevatorMotorWSConfig = new SparkMaxConfig();
        elevatorMotorWSConfig.idleMode(IdleMode.kCoast).inverted(true);
        elevatorMotorWSConfig.closedLoop.pid(Constants.Swerve.elevator.kP, Constants.Swerve.elevator.kI, Constants.Swerve.elevator.kD);
        //TODO elevatorMotorWSConfig.encoder.velocityConversionFactor(())
        elevatorMotorWS.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        elevatorMotorEncoder = elevatorMotor.getEncoder();

        // Set the position to zero
        elevatorMotorEncoder.setPosition(0);



        //Initializes encoder for intake's arm



        // Set Conversion factor for Encoder -> degrees 

    }
    
   
    public Command Stop(){
        return run(()->{
            elevatorMotor.set(0);
            elevatorMotorWS.set(0);
        });
    }

    public Rotation2d getPos(){
        Rotation2d posElevatorMotor = Rotation2d.fromDegrees(elevatorMotorEncoder.getPosition()); 
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
    public void periodic(){
        SmartDashboard.putNumber("IntakePos", getPos().getDegrees());
    }
}
