
package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.MathUtils;


public class Climber extends SubsystemBase{
    private SparkMax climbMotor;
    private RelativeEncoder climbMotorEncoder;
    private SparkClosedLoopController climbMotorPID;
    private PIDController elevatorMotorPID;
    private PWM trapServo;
    private PWM ratchetServo;
    private boolean climbMotorDirection;
    private boolean idleClimb;
    public Climber() {
        // initialies all the variables and constants
        climbMotor = new SparkMax(frc.robot.Constants.Climber.rotMotorID, MotorType.kBrushless );
        trapServo = new PWM(Constants.Swerve.trapServoID);
        ratchetServo = new PWM(Constants.Swerve.ratchetServoID);
        ratchetServo.setPosition(0);
        SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
        climbMotorConfig.idleMode(IdleMode.kBrake).inverted(false);
        climbMotorConfig.closedLoop.pid(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD);
        elevatorMotorPID = new PIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD);
        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotorEncoder = climbMotor.getEncoder();
        climbMotorEncoder.setPosition(0);
        climbMotorPID = climbMotor.getClosedLoopController();
    }


    public double summonPosition(){
         //RPM
        return climbMotorEncoder.getPosition();
    }
     public Rotation2d getPos(){
        Rotation2d posClimb = Rotation2d.fromDegrees(-climbMotorEncoder.getPosition()); 
        return posClimb;
    }

    public Rotation2d getError(Rotation2d goal){
        Rotation2d currPos = getPos();
        Rotation2d error = Rotation2d.fromDegrees(currPos.getDegrees() - goal.getDegrees());
        return error;
    }

    public void GoTo(Rotation2d goal){
        climbMotorPID.setReference(
            goal.getDegrees(), 
            ControlType.kPosition, ClosedLoopSlot.kSlot0
        );
    }
    private boolean atSetpoint = false;

    public void reachGoal(double goal){
        double voltageOutput = MathUtils.clamp(elevatorMotorPID.calculate(summonPosition(), goal), 7, -7);
        climbMotor.setVoltage(voltageOutput);
    }
    public Command arise(double goal){
        return run(()->{reachGoal(goal);}).until(()->Math.abs(summonPosition() - goal) < Constants.Climber.tolerance).andThen(()->ratchetOn());
    }

    /*public Command moveTo(BooleanSupplier amp, BooleanSupplier speaker, DoubleSupplier trigger){
        final double[] v = new double[]{(amp.getAsBoolean())? flywheel.AMP: flywheel.SPEAKER, (amp.getAsBoolean())? flywheel.AMP: flywheel.SPEAKER};
        SmartDashboard.putNumber("trigger value", trigger.getAsDouble());
        System.out.println(trigger.getAsDouble());
        if (trigger.getAsDouble() > 0.2){
            System.out.println("running flywheel");
            return runOnce(()->{
                SmartDashboard.putNumber("Velocity Goal", v[0]);
                //System.out.println(v[0]);
                }).andThen(run(
                    () -> GoTo(
                        v[0], v[1]
                    )
                ).until(
                    ()->(
                        Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance
                        && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance
                    )).andThen(runOnce(()->{
                        atSetpoint = true;
                    }))
            );
        }
        else{
            return runOnce(()->{});
        }
       
    }*/
    public Command stop(){
        return run(()->{climbMotor.set(0); climbMotor.set(0);});
    }

    /*public Command speakerShot(){
        return run(()->{top.set(0.4); bott.set(0.4);});
    }*/

    public Command moveTo(Rotation2d goal){
        
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Goal", goal.getDegrees());
        }).andThen(run(
            () -> GoTo(goal)
        ).until(
            ()->(
                Math.abs(getError(goal).getDegrees()) < Constants.Climber.tolerance
            ))
        );
    }
    


        /* .until(
            ()->(
                (Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance
                && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance) && !trigger.getAsBoolean()
            )).andThen(runOnce(()->{
                if (idleState){
                    atSetpoint = false;
                }else{
                    atSetpoint = true;
                }
            }))*/
        
    public Command ratchetControl(BooleanSupplier pos){
        if(pos.getAsBoolean()){
            return run(()->{ratchetServo.setPosition(0);});
        }else{
            return run(()->{ratchetServo.setPosition(1);});
        }
        // ratchetServo.setPosition((pos.getAsBoolean()) ? 0 : 1);
        // return pos.getAsBoolean();
    }
    public void ratchetOn(){
        // trapServo.setSpeed(1);
        ratchetServo.setPosition(1);
    }
    public void ratchetOff(){
        ratchetServo.setPosition(0);
    }
    public void trapOn(){
        // trapServo.setSpeed(1);
        trapServo.setPosition(1);
    }
    public void trapOff(){
        trapServo.setSpeed(0);
    }

    
    public Command moveTill(double goalPos, boolean idleState){
        double p  = goalPos;
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Climber Goal", p);
        }).andThen(run(
            () -> {GoTo((new Rotation2d(climbMotorEncoder.getPosition() + p)));}
        ).until(
            ()->(
                (Math.abs(getError(Rotation2d.fromDegrees(p)).getDegrees()) < Constants.Climber.tolerance)
            )).andThen(runOnce(()->{
                if (idleState){
                    atSetpoint = false;
                }else{
                    atSetpoint = true;
                }
                })
            ).andThen(runOnce(() -> {
                double getPosition = this.summonPosition();
                System.out.println(getPosition);
            })).until(
                () -> (
                    Math.abs(this.getError(Rotation2d.fromDegrees(climbMotorEncoder.getPosition())).getDegrees()) < Constants.Climber.tolerance
                )
            ).andThen(runOnce(() -> {
                if (idleState) {
                    atSetpoint = false;
                } else {
                    atSetpoint = true;
                }
            }))
        );
    }
    
    // Manual Controls:
     public void Down(){
        climbMotor.set(-.5);
        climbMotorDirection = false;
        idleClimb = false;
    }
    public void Up(){
        climbMotor.set(.5);
        climbMotorDirection = true;
        idleClimb = false;
    }
    public void Stop(){
        climbMotor.stopMotor();
        idleClimb = true;
    }
    public Command manualControl(BooleanSupplier up, BooleanSupplier down, BooleanSupplier ratchet){
        return run(()->{
            if (ratchet.getAsBoolean()){
                ratchetOff();
            }
            else{
                ratchetOn();
            }
            if (up.getAsBoolean()){
               Up(); 
               System.out.print("going up");
            }
            else if (down.getAsBoolean()){
                Down();
            }
            else{
                Stop();
            }
        });

    }

    
    public void periodic(){
        // ratchetOff();
        // if (!idleClimb){
        //     if (climbMotorDirection){

        //         ratchetServo.setPosition(1);
        //     }
        //     else{
        //         ratchetServo.setPosition(0);
        //     }
        // }
        // else{
        //     ratchetServo.setPosition(0);
        // }
        // System.out.println(ratchetServo.getPosition());
        SmartDashboard.putNumber("Climb Position", summonPosition());
        // SmartDashboard.putBoolean("Ratchet Locked?", ratchetControl(()->SmartDashboard.getBoolean("Ratchet Locked?", true)));
    }


}




