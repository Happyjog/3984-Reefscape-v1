package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;
public class autoScoreSequence extends Command{
    private final Swerve s_Swerve;
    private final Elevator s_Elevator;
    private final Supplier<Pose2d> poseProvider;
    private final PIDController moveXController = new PIDController(2.1, 0, 0);
    private final PIDController moveYController = new PIDController(2.1, 0, 0);
    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private boolean isDone;
    private boolean earlyStop;
    private double x;
    private double y;
    private long curr_tag_in_view;
    private double elevator_height;
    public autoScoreSequence(
        Swerve s_Swerve,
        Elevator s_Elevator,
        Supplier<Pose2d> poseProvider) {
        this.s_Swerve = s_Swerve;
        this.s_Elevator = s_Elevator;
        this.poseProvider = poseProvider;
        // this.amount_offset = amount_offset;
        // moveXController.setTolerance(0.05);
        // moveYController.setTolerance(0.05);
        addRequirements(s_Swerve, s_Elevator);
    }
    
    @Override
    public void initialize() {
        isDone = false;
        earlyStop = false;
        s_Swerve.togglePreciseTargeting(true);
        curr_tag_in_view = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(-1);
        // TODO change key
        String elevator_level = NetworkTableInstance.getDefault().getTable("cumber").getStringTopic("Level").toString();//.getString("Level", "0");
        if (elevator_level.equals("L4") || elevator_level.equals("R4")){
            elevator_height = Constants.Elevator.elevatorShaft.kLEVEL4;
        }
        else if (elevator_level.equals("L3") || elevator_level.equals("R3")){
            elevator_height = Constants.Elevator.elevatorShaft.kLEVEL3;
        }
        else if (elevator_level.equals("L2")  || elevator_level.equals("R2")){
            elevator_height = Constants.Elevator.elevatorShaft.kLEVEL2;
        }
        else if (elevator_level.equals("0")){
            elevator_height = Constants.Elevator.elevatorShaft.kLEVEL1;
        }

        if (curr_tag_in_view < 0){
            isDone = true;
            earlyStop = true;
            System.out.println("No apriltag");
        }
        else{
            var roboPose = poseProvider.get();
            Pose2d tag_pose = new Pose2d();
            if (curr_tag_in_view > 0){
                tag_pose = layout.getTagPose((int)(curr_tag_in_view)).get().toPose2d();

            }
            double tag_x = tag_pose.getX();
            double tag_y = tag_pose.getY();
            Rotation2d tag_theta = tag_pose.getRotation().minus(Rotation2d.fromDegrees(90));
            double forward_offset = -1;
            double lateral_offset = 0;
            // double x_offset = tag_x + forward_offset * tag_theta.getCos() - lateral_offset * tag_theta.getSin();
            // double y_offset = tag_y + forward_offset * tag_theta.getSin() + lateral_offset * tag_theta.getCos();
            double x_offset = tag_x + forward_offset * tag_theta.getSin() - lateral_offset * tag_theta.getCos();
            double y_offset = tag_y - forward_offset * tag_theta.getCos() - lateral_offset * tag_theta.getSin();
            double diff_x = roboPose.getX() - x_offset;
            double diff_y = roboPose.getY() - y_offset;
            System.out.println("x_diff"+ diff_x + "y_dff" + diff_y + "angle" + roboPose.getRotation().minus(tag_pose.getRotation()).getDegrees());
            // s_Swerve.plots(new Pose2d(x_offset, y_offset,tag_theta));

            // System.out.println("HEEEYEEYYEYEYE: " + diff_x + "and"+ diff_y)
            x = x_offset;
            y = y_offset;
            // x = tag_pose.getX();
            // y = tag_pose.getY();

        }
        
        
    }
  
    @Override
    public void execute() {
        System.out.println("Executed main loop: "+ isDone);
        var robotPose2d = poseProvider.get();
        if (isDone){
            return;
        }
        double delx = x - robotPose2d.getX();
        double dely = y - robotPose2d.getY();
        System.out.println("ID: "+ curr_tag_in_view + " diff x: " + delx + " diff y: " + dely);
        // Output Volts is capped at 2 to prevent brownout
        double xOutput = MathUtil.clamp(moveXController.calculate(-1*delx), -3, 3);
        double yOutput = Math.min(moveYController.calculate(-1*dely), 3);
        s_Swerve.drive(new Translation2d(xOutput, yOutput), 0, true, true);
        if (Math.abs(delx) < 0.03 && Math.abs(dely) < 0.03){
            isDone = true;
        }
        else{
            isDone = false;
        }
        
    }
    @Override
    public boolean isFinished(){
        return isDone;
    }
    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
        if (!earlyStop){
            s_Elevator.setHeight(elevator_height);
        }
    }
    
}

