// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {

        public static final double stickDeadband = 0.1;
        public static final boolean invertGyro = false;// false

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final Translation2d f1ModuleOffset = new Translation2d(trackWidth / 2, trackWidth / 2);
        public static final Translation2d f2ModuleOffset = new Translation2d(trackWidth / 2, -trackWidth / 2);
        public static final Translation2d b1ModuleOffset = new Translation2d(-trackWidth / 2, trackWidth / 2);
        public static final Translation2d b2ModuleOffset = new Translation2d(-trackWidth / 2, -trackWidth / 2);
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0);
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.005;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.025;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveKFF = 0.0;

        /* TODO Drive Motor Characterization Values */
        public static final double driveKS = 0.667;// 0.11563;
        public static final double driveKV = 2.44;// 2.656;
        public static final double driveKA = 0.27;// 0.276;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; // meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false; // true
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; 
        // TODO
        /* PWM PORTS */
        public static final int ratchetServoID = 0;
        public static final int trapServoID = 1;
        /* DIO PORTS */
        public static final int limitSwitch1ID = 4;
        public static final int limitSwitch2ID = 5;
        public static final int throBoreAbsoluteChannel = 0;
        public static final int throBoreRelativeChannel1 = 1;
        public static final int throBoreRelativeChannel2 = 2;

        

        /* Module Specific Constants */
        
        /* Front Left Module - Module 0 */
        // GOOD
        public static final class Mod0 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.58984375);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        

        /* Back Left Module - Module 1 */
        public static final class Mod2 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(24.69921875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 2 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(158.64257815);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        
        /* Back Right Module - Module 3 */
        // GOOD
        public static final class Mod3 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(218.49609315);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }
    
    public static final class Elevator {
        // TODO Set elevator constants 

        public static final class elevatorShaft {
            public static final int shaftMotorID = 7;
            public static final int shaftMotorWSID = 16;
            public static final int laserCan1ID = 17; //TODO
            public static final int laserCan2ID = 18; //TODO
            public static final double kP = 0.001; // TODO
            public static final double kI = 0.00003; // TODO
            public static final double kD = 0.00001; // TODO
            public static final double kS = 0; // TODO
            public static final double kG = 0.9; // TODO
            public static final double kV = 0; // TODO
            public static final double kA = 0; // TODO
            public static final double kMaxVelocity = 0; // TODO
            public static final double kMaxAcceleration = 0; // TODO
            public static final double kElevatorMotorRampRate = 0.1; 
            public static final double kErrorTolerance = 100;
            public static final double kFULLEXTENSION = -2022; // TODO
            public static final double kLEVEL1 = 0; 
            public static final double kLEVEL2 = 2210;
            public static final double kLEVEL3 = 5186;
            public static final double kLEVEL4 = 9750;
            public static final TrapezoidProfile.Constraints kElevatorConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

        }
        
    }
    public static final class Outtake{
        public static final int outtakeMotorID = 5;
        public static final int outtakeMotorWSID = 8;
        public static final double kLaserDistCali1 = 100;
        public static final double kLaserDistCali2 = 100;
    }
    public static final class Climber {
        public static final int rotMotorID = 9; // climber left
        public static final double tolerance = 1.0;
        public static final double kP = 0.01;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double gearRatio = 25;
        public static final Rotation2d[] FULLEXTENSION = new Rotation2d[] { Rotation2d.fromDegrees(-2022),
                Rotation2d.fromDegrees(2022) };
        public static final double kClimberOut = 151.82936096191406;
        public static final double kClimberIn = 0;
    }

    // TODO remove
    public static final class Flywheel {
        public static final double FWDiameter = 4.0; // Inches NEED TO CONVERT
        public static final double tolerance = 1; // Meters per second // changed from 1 to 0.25 - bentley 11/13/24

        public static final class FWtop {// TODO PID
            public static final int FWid = 18;
            public static final double kP = 0.1;// 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kV = 0.01;
            public static final double gearRatio = 1;
        }

        public static final class FWbott { // TODO PID
            public static final int FWid = 19;
            public static final double kP = 0.1;// 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kV = 0.01;
            public static final double gearRatio = 1;
        }

        // AMP and SPEAKER velocity presets
        public static final double AMP = 10;
        public static final double SPEAKER = 15;// 15; // changed from 50 to 15 - bentley 11/13/24

    }
    // TODO remove
    public static final class intake {
        public static final double tolerance = 0.1;

        public static final class intakeArm {
            public static final int rotMotorID = 12; // TODO
            public static final double kP = 0.005;
            public static final double kI = 0;
            public static final double kD = 0.0001;
            public static final double gearRatio = 100;

        }

        public static final class intakeMotor {
            public static final int IDR = 0;
            public static final int IDL = 1;

        }

        public static final Rotation2d INTAKE = Rotation2d.fromDegrees(-200);
        public static final Rotation2d AMPSHOT = Rotation2d.fromDegrees(-94 + 10 + 5);
        public static final Rotation2d IDLE = Rotation2d.fromDegrees(0);

    }
    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.000000000000001;// 0.2;//0.1;
        public static final double kPYController = 0.000000000000001;// 0.2;// 0.1;
        public static final double kPThetaController = 7.5;
        public static final Translation2d GOALRIGHT = new Translation2d(1, -1);
        public static final Translation2d GOALLEFT = new Translation2d(1, 1);
        public static final Translation2d GOALMIDDLE = new Translation2d(1, 0);
        
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
