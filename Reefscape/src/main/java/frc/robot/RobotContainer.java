// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;



// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WaitForCoralCommand;
import frc.robot.commands.moveToOffset;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Spitter;
import frc.robot.Constants.Elevator.elevatorShaft;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands
  private final CommandXboxController driver = new CommandXboxController(1);
  // private final CommandXboxController second = new CommandXboxController(0);
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;
  // private final int triggerLAxis = XboxController.Axis.kLeftTrigger.value;
  // private final int triggerRAxis = XboxController.Axis.kRightTrigger.value;
  /*
   * private final JoystickButton zeroGyro =
   * new JoystickButton(driver, XboxController.Button.kY.value);
   * private final JoystickButton aim =
   * new JoystickButton(driver, XboxController.Button.kA.value);
   * private final JoystickButton speaker =
   * new JoystickButton(driver, XboxController.Button.kX.value);
   * private final JoystickButton amp =
   * new JoystickButton(driver, XboxController.Button.kB.value);
   * private final JoystickButton lolintake =
   * new JoystickButton(driver, XboxController.Button.kRightBumper.value);
   * private final JoystickButton slow =
   * new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
   */
  // private final Flywheel fwheel = new Flywheel();
  // private final PhotonCamera cam = new PhotonCamera("Global_Shutter_Camera");
  // private final PhotonCamera cam2 = new
  // PhotonCamera("Microsoft_LifeCam_HD-3000");
  // private final SendableChooser<Command> autoChooser;
  private final Swerve s_Swerve = new Swerve();
  private final Elevator s_Elevator = new Elevator();
  private final Climber s_Climber = new Climber();
  private final Spitter s_Spitter = new Spitter();

  // TODO Add offsets
  private final moveToOffset moveL = new moveToOffset(s_Swerve, s_Swerve::getPose);
  private final moveToOffset moveR = new moveToOffset(s_Swerve, s_Swerve::getPose);

  /* The container for the robot. subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Pathplanner commands
    // NamedCommands.registerCommand("WaitForCoral", s_Spitter.WaitForCoralCommand());

    // NamedCommands.registerCommand("ScoreL4", moveL.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL4)));
    // NamedCommands.registerCommand("ScoreL3", moveL.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL3)));
    // NamedCommands.registerCommand("ScoreL2", moveL.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL2)));
    // NamedCommands.registerCommand("ScoreL1", moveL.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL1)));

    // NamedCommands.registerCommand("ScoreR4", moveR.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL4)));
    // NamedCommands.registerCommand("ScoreR3", moveR.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL3)));
    // NamedCommands.registerCommand("ScoreR2", moveR.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL2)));
    // NamedCommands.registerCommand("ScoreR1", moveR.andThen(s_Elevator.setHeight(elevatorShaft.kLEVEL1)));

    // autoChooser = AutoBuilder.buildAutoChooser();
    // System.out.println(AutoBuilder.getAllAutoNames());
    // SmartDashboard.putData("AutoChooser", autoChooser);
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            () -> driver.leftBumper().getAsBoolean(),
            () -> driver.x().getAsBoolean()));
    // TODO Remove manual control later
    s_Elevator.setDefaultCommand(
        s_Elevator.manualShaftControl(
            () -> driver.leftBumper().getAsBoolean(),
            () -> driver.rightBumper().getAsBoolean()));
    
    // s_Climber.setDefaultCommand(
    //   s_Climber.manualControl(
    //   ()-> driver.leftBumper().getAsBoolean(),
    //   ()-> driver.rightBumper().getAsBoolean()
    //   ));

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driver.x().onTrue(new InstantCommand(() -> s_Swerve.setAbsolute()));
    driver.a().onTrue(new InstantCommand(() -> s_Spitter.OuttakeOut()));
    driver.b().onTrue(s_Elevator.setHeight(Constants.Elevator.elevatorShaft.kLEVEL4));
    driver.start().onTrue(s_Elevator.setHeight(Constants.Elevator.elevatorShaft.kLEVEL1));
    // driver.a().onTrue(s_Climber.ratchetControl());
    // driver.b().onTrue(translateApriltag);
    // driver.b().onTrue(s_Swerve.moveTo(new Pose2d(s_Swerve.getPose().getX()+1,
    // s_Swerve.getPose().getY(), s_Swerve.getPose().getRotation())));
    // driver.x().onTrue(new InstantCommand(()->s_Swerve.setAbsolute()));
    // second.rightTrigger(0.3).whileTrue(intake.Out());
    // second.leftTrigger(0.3).whileTrue(fwheel.moveTo(flywheel.SPEAKER,
    // flywheel.SPEAKER, false, ()->driver.leftTrigger(0.3).getAsBoolean(), note));
    // second.leftTrigger(0.3).whileTrue(fwheel.speakerShot());
    // second.a().whileTrue(intake.In());

    // second.x().whileTrue(intake.moveTo(Constants.Swerve.intake.IDLE, false));
    // second.y().whileTrue(intake.moveTo(Constants.Swerve.intake.AMPSHOT, false));
    // second.b().whileTrue(intake.moveTo(Constants.Swerve.intake.INTAKE, true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();//autoChooser.getSelected();
  }
  // public Command testPath(){return new PathPlannerAuto("testAuto");}
  // // Left side autos
  // public Command Left1Note(){return new PathPlannerAuto("Left1Note");}
  // public Command Left2Note(){return new PathPlannerAuto("Left2Note");}
  // //public Command Left3Note(){return new PathPlannerAuto("Left3Note");}
  // //public Command Left3NoteFar(){return new PathPlannerAuto("Left3NoteFar");}
  // //public Command Left4Note(){return new PathPlannerAuto("Left4Note");}
  // // Mid side autos
  // public Command Mid1Note(){return new PathPlannerAuto("Mid1Note");}
  // public Command Mid2Note(){return new PathPlannerAuto("Mid2Note");}
  // //public Command Mid3NoteFar(){return new PathPlannerAuto("Mid3NoteFar");}
  // //public Command Mid4Note(){return new PathPlannerAuto("Mid4Note");}
  // //public Command MidLeft3Note(){return new PathPlannerAuto("MidLeft3Note");}
  // //public Command MidRight3Note(){return new
  // PathPlannerAuto("MidRight3Note");}
  // //Right side autos
  // public Command Right1Note(){return new PathPlannerAuto("Right1Note");}
  // public Command Right2Note(){return new PathPlannerAuto("Right2Note");}
  // public Command Right2NoteFar(){return new PathPlannerAuto("Right2NoteFar");}
  // public Command Right3NoteFar(){return new PathPlannerAuto("Right3NoteFar");}
}
