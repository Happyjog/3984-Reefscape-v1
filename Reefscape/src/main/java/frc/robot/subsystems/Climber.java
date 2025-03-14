
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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase{
    private SparkMax climbMotor;
    private RelativeEncoder climbMotorEncoder;
    private SparkClosedLoopController climbMotorPID;
    public Climber() {
        // initialies all the variables and constants
        climbMotor = new SparkMax(frc.robot.Constants.Swerve.climber.rotMotorID, MotorType.kBrushless );
        SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
        climbMotorConfig.idleMode(IdleMode.kBrake).inverted(false);
        climbMotorConfig.closedLoop.pid(Constants.Swerve.flywheel.FWtop.kP, Constants.Swerve.flywheel.FWtop.kI, Constants.Swerve.flywheel.FWtop.kD);
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
                Math.abs(getError(goal).getDegrees()) < Constants.Swerve.climber.tolerance
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
        );
    }
    public Command moveTill(double vtop, double vbott, boolean idleState){
        double[] v  =new double[]{vtop, vbott};
        //Rotation2d[] a = new Rotation2d[]{Rotation2d.fromDegrees(Sangle)/*(49.26)/*getAngles(x, y)[0]*/, Rotation2d.fromDegrees(Jangle)};/*(61.97)};// getAngles(x, y)[1]};*/
        return runOnce(()->{
            SmartDashboard.putNumber("Shoulder Goal", v[0]);
            SmartDashboard.putNumber("Joint Goal", v[1]);
        }).andThen(run(
            () -> {GoTo(v[0], v[1]);
            double[] getVelocity = this.getVelocity();
            System.out.println(getVelocity);
            }


        ) .until(
            ()->(
                (Math.abs(getErrors(v)[0]) < Constants.Swerve.flywheel.tolerance
                && Math.abs(getErrors(v)[1]) < Constants.Swerve.flywheel.tolerance)
            )).andThen(runOnce(()->{
                if (idleState){
                    atSetpoint = false;
                }else{
                    atSetpoint = true;
                }
            }))
        );
    }
    public void periodic(){
        SmartDashboard.putBoolean("Ready To Shoot?", atSetpoint);
        SmartDashboard.putNumber("lywhell Vellovity", getVelocity()[0]);
    }


}




