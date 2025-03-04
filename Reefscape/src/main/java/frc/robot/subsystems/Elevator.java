package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase{
    
    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorMotorEncoder;
    private SparkPIDController elevatorMotorPID;
    private Spark intakeMotorL;
    private Spark intakeMotorR;

    public Elevator(){

        elevatorMotor = new CANSparkMax(
            Constants.elevator.rotMotorID,
            MotorType.kBrushless
        );



        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        //Initializes encoder for intake's arm
        
        elevatorMotorEncoder = elevatorMotor.getEncoder(
            SparkRelativeEncoder.Type.kHallSensor, 
            42
        );


        // Set Conversion factor for Encoder -> degrees
        elevatorMotorEncoder.setPositionConversionFactor(
            360.0/Constants.elevator.gearRatio //TODO
        ); 
        // Set Velocity Conversion factor -> degrees/second
        elevatorMotorEncoder.setVelocityConversionFactor(
            (360 / Constants.elevator.gearRatio) / 60.0  //TODO
        );
        // Set the position to zero
        elevatorMotorEncoder.setPosition(0);

        //Initialize elevator PID
        elevatorMotorPID = elevatorMotor.getPIDController();
        elevatorMotorPID.setP(Constants.elevator.kP);
        elevatorMotorPID.setI(Constants.elevator.kI);
        elevatorMotorPID.setD(Constants.elevator.kD);

        elevatorMotor.burnFlash();
    }
    
   
    public Command Stop(){
        return run(()->{
            intakeMotorL.stopMotor();
            intakeMotorR.stopMotor();
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
            ControlType.kPosition, 0
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
