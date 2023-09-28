// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeSubsystem;
import frc.robot.commands.auton.Center;
import frc.robot.commands.auton.SOS;
import frc.robot.commands.auton.bump;
import frc.robot.commands.auton.nonbump;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  private final PS4Controller operator = new PS4Controller(1);
  // private final Joystick logitech = new Joystick(1);

  /* Buttons */
  private final JoystickButton dCircle = new JoystickButton(driver, PS4Controller.Button.kCircle.value);


  private final JoystickButton oCross = new JoystickButton(operator, PS4Controller.Button.kCross.value);
  private final JoystickButton oTriangle = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton oRT = new JoystickButton(operator, PS4Controller.Button.kR2.value);
  private final JoystickButton oLT = new JoystickButton(operator, PS4Controller.Button.kL2.value);
  private final JoystickButton oCircle = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton oSquare = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
  private final JoystickButton oLB = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton oRB = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton oDpadUp = new JoystickButton(operator, PS4Controller.Axis.kLeftX.value
  );

  // public static final int kGamepadButtonA = 1; // Bottom Button
	// public static final int kGamepadButtonB = 2; // Right Button
	// public static final int kGamepadButtonX = 3; // Left Button
	// public static final int kGamepadButtonY = 4; // Top Button
	// public static final int kGamepadButtonShoulderL = 5;
	// public static final int kGamepadButtonShoulderR = 6;
	// public static final int kGamepadButtonBack = 7;
	// public static final int kGamepadButtonStart = 8;
	// public static final int kGamepadButtonLeftStick = 9;
	// public static final int kGamepadButtonRightStick = 10;

  


  // private final JoystickButton oDpadUp = new JoystickButton(operator, PS4Controller.Button..value);

  // private final JoystickButton shoot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  private final frc.robot.subsystems.IntakeSubsystem intake = new frc.robot.subsystems.IntakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  
  /* Commands */
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  // private final Command c_shoot = new Shoot(shooter, 1850.0);


  private enum Autons { Nothing, Bump, NonBump, Center, CenterCubeHP, CenterCubeWall}
  private SendableChooser<Autons> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    setDefaultCommands();
    configureButtonBindings();
    configureAuton();
    loadTrajectories();
  }

  private void configureButtonBindings() {
    dCircle.onTrue(c_zeroGyro);

    oRT.onTrue(new InstantCommand( () -> intake.intakeDefSpeed() ));
    oLT.onTrue(new InstantCommand( () -> intake.outtakeDefSpeed() ));
    oRT.onFalse(new InstantCommand( () -> intake.stop() ));
    oLT.onFalse(new InstantCommand( () -> intake.stop() ));

    oRB.onTrue(new InstantCommand( () -> arm.manualArmUp() ));
    oLB.onTrue(new InstantCommand( () -> arm.manualArmDown() ));
    oRB.onFalse(new InstantCommand( () -> arm.stop() ));
    oLB.onFalse(new InstantCommand( () -> arm.stop() ));

    oCross.onTrue(new InstantCommand( () -> arm.armGround() )); //armGround()
    oTriangle.onTrue(new InstantCommand( () -> arm.armMid() ));
    oSquare.onTrue(new InstantCommand( () -> arm.findZero() ));    
  
  }

  public void checkOperatorPOV(){
    if(operator.getPOV() == 0){
      arm.manualArmUp();
    }
    else if(operator.getPOV() == 180){
      arm.manualArmDown();
    }

  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveTeleop(
        drivetrain,
        driver,
        true, true
      )
    );

    // shooter.setDefaultCommand(new IdleShooter(shooter));
  }

  private void configureAuton(){
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption("Bump", Autons.Bump);
    autonChooser.addOption("Non-Bump", Autons.NonBump);
    // autonChooser.addOption("Center HP", Autons.CenterCubeHP);
    // autonChooser.addOption("Center Wall", Autons.CenterCubeWall);
    autonChooser.addOption("Center", Autons.Center);

    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    tab.add("Auton Chooser", autonChooser);

  }
  private void loadTrajectories() {

  }

  public Command getAutonomousCommand() {
    switch (autonChooser.getSelected()){
      case Nothing:
        return new WaitCommand(15);

      case Bump:
        // double velocity = 4;
        // double maxaccel = 2;
        // return new bump(drivetrain, intake, arm, 
        // TrajectoryHelper.loadHolonomicPathPlannerTrajectory("Bump1",velocity,maxaccel, false),
        //  null, null, null);
        return new SOS(drivetrain, intake, arm, 
        TrajectoryHelper.loadHolonomicPathPlannerTrajectory("SOS-Bump",3,2, false));

      case NonBump:
        // double vel = 6.5;
        // double accel = 2.5;

        // return new nonbump(drivetrain, intake, arm, 
        // TrajectoryHelper.loadHolonomicPathPlannerTrajectory("NonBump1",vel,accel, false), 
        // TrajectoryHelper.loadHolonomicPathPlannerTrajectory("NonBump2",vel,accel, false), 
        // TrajectoryHelper.loadHolonomicPathPlannerTrajectory("NonBump3",vel,accel, false), 
        // TrajectoryHelper.loadHolonomicPathPlannerTrajectory("NonBump4",vel,accel, false));

        return new SOS(drivetrain, intake, arm, 
        TrajectoryHelper.loadHolonomicPathPlannerTrajectory("SOS-NonBump",3,2, false));


      case Center:
        return new Center(drivetrain, intake, arm, 
        TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_Center", 1.8, 1, false));

      default:
        return new WaitCommand(15);

    }
  }
}
