// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.swerve.SwerveModuleConstants;

public final class Constants {
    public static final class SwerveDrivetrain {

        /* Gyro */
        public static final int GYRO_ID = 50;
        public static final boolean INVERT_GYRO = false;

        /* Drivetrain */
        public static final double TRACK_WIDTH          = Units.inchesToMeters(15.125);
        public static final double WHEEL_BASE           = Units.inchesToMeters(15.125);
        public static final double WHEEL_DIAMETER       = Units.inchesToMeters(3.58);
        public static final double WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP   = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (6.0 / 1.0);  // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (12.0 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(  WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d(  WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0)
        );

        /* Current Limiting */
        public static final int ANGLE_CONTINUOUS_CL = 25;
        public static final int ANGLE_PEAK_CL       = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL       = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_kP = 0.6;   // 0.6
        public static final double ANGLE_kI = 0.0;   // 0.0
        public static final double ANGLE_kD = 12.0;   // 12.0
        public static final double ANGLE_kF = 0.0;   // 0.0

        /* Drive Motor PID Values */
        public static final double DRIVE_kP = 0.10;  // 0.10
        public static final double DRIVE_kI = 0.0;   // 0.0
        public static final double DRIVE_kD = 0.0;   // 0.0
        public static final double DRIVE_kF = 0.0;   // 0.0

        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FF_kS    = (0.632 / 12);     // 0.667 --- divide by 12 to convert from volts to percent output for CTRE
        public static final double FF_kV    = (0.0514 / 12);    // 2.44
        public static final double FF_kA    = (0.00337 / 12);   // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED            = 4.5;  // m/s
        public static final double MAX_ANGULAR_VELOCITY = 11.5; // m/s

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID  = 1;
            public static final int ANGLE_MOTOR_ID  = 2;
            public static final int CAN_CODER_ID    = 3;
            public static final double ANGLE_OFFSET = 45.791;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID  = 5;
            public static final int ANGLE_MOTOR_ID  = 4;
            public static final int CAN_CODER_ID    = 6;
            public static final double ANGLE_OFFSET = 138.867;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID  = 11;
            public static final int ANGLE_MOTOR_ID  = 10;
            public static final int CAN_CODER_ID    = 12;
            public static final double ANGLE_OFFSET = 218.320;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID  = 7;
            public static final int ANGLE_MOTOR_ID  = 9;
            public static final int CAN_CODER_ID    = 8;
            public static final double ANGLE_OFFSET = 88.682;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class IntakeSubsystem {
        public static final int MOTOR_ID = 14;
        public static final double CUBE_DEFAULT_INTAKE_SPEED = 0.35;
        public static final double CUBE_DEFAULT_OUTTAKE_SPEED = 0.74; //0.74

    }

    public static final class ArmSubsystem {
        public static final int MID_HEIGHT = -25595; //-22000
        public static final int GROUND_HEIGHT = 50;
        public static final int PARTIAL_HEIGHT = -12500;
        public static final int DIO_PORT = 1;
        public static final double MANUAL_SPEED = 0.1;

        public static final int MOTOR_ID = 15;
    }

    public static final class Auton {
        public static final double MAX_SPEED_MPS            = 7.0;    // meters per second
        public static final double MAX_ACCELERATION_MPSS    = 5.0;    // meters per second squared

        public static final double MAX_ANGULAR_SPEED_RPS    = 2 * Math.PI;      // radians per second
        public static final double MAX_ANGULAR_SPEED_RPSS   = 2 * Math.PI;      // radians per second squared

        // public static final PIDController PX_CONTROLLER = new PIDController(5.25, 1, 0.4);
        // public static final PIDController PY_CONTROLLER = new PIDController(5.25, 1, 0.4);
        public static final PIDController PX_CONTROLLER = new PIDController(6.0, 0, 0.1);
        public static final PIDController PY_CONTROLLER = new PIDController(6.0, 0, 0.1);
        // public static final double PTHETA_CONTROLLER    = 1.0;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPSS);

        // public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(.13, 0, .39, THETA_CONTROLLER_CONTRAINTS);
        // public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(10.0, 0.0, 0.0, THETA_CONTROLLER_CONTRAINTS);
        public static final PIDController THETA_CONTROLLER = new PIDController(4.0, 0.0, 0.0); //kP 10

    }

}
