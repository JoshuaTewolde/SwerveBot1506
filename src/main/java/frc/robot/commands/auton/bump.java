// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.intake.JustIntake;
import frc.robot.commands.intake.JustOuttake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.TapAutoCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

import frc.robot.commands.arm.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class bump extends SequentialCommandGroup {
  /** Creates a new OnePathplannerTest. */
  // List<Waypoint> waypoints = new ArrayList<Waypoint>(Arrays.asList(autopaths))
  // PathPlanner.loadPathGroup("NonBump", Constants.Auton.MAX_SPEED_MPS, Constants.Auton.MAX_ANGULAR_SPEED_RPSS);
//   List<EventMarker> markers = new ArrayList<EventMarker>();
  public bump(SwerveDrivetrain drivetrain, IntakeSubsystem intake,
  ArmSubsystem arm, PathPlannerTrajectory trajectory1, 
  PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3, PathPlannerTrajectory trajectory4) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    boolean isFirstPath = true;

    getsize();
    // new PathPlannerTrajectory(autopaths, markers, new PathConstraints(Constants.Auton.MAX_SPEED_MPS, Constants.Auton.MAX_ACCELERATION_MPSS), false, false);

    addCommands(
      // new RunPathPlannerTrajectory2(drivetrain, trajectory(1), true)
      
      new SequentialCommandGroup(
        //drop cone
        new DropCube(drivetrain, intake, arm),
        //go to first cube, drop arm once past 180 degrees
        
        // new RunPathPlannerTrajectory2(drivetrain, PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory1, DriverStation.getAlliance()), true),
        new ParallelDeadlineGroup(
          new RunPathPlannerTrajectory2(drivetrain, PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory1, DriverStation.getAlliance()), true),
          
          new dropArmSpecific2(arm, drivetrain),
          new JustIntake(intake).withTimeout(0.1)
        ),
        new ParallelCommandGroup(
          new RunPathPlannerTrajectory2(drivetrain, PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory2, DriverStation.getAlliance()), isFirstPath),
          new armPartial(arm).withTimeout(0.3),
          new TapAutoCube(intake).withTimeout(0.1)
        ),
        new armLow(arm).withTimeout(0.3),
        new JustOuttake(intake).withTimeout(0.1),
        new StopIntake(intake).withTimeout(0.01),
        new armMid(arm).withTimeout(0.4),

        new ParallelDeadlineGroup(
          new RunPathPlannerTrajectory2(drivetrain, PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory3, DriverStation.getAlliance()), isFirstPath),
          new dropArmSpecific(arm, drivetrain),
          new JustIntake(intake).withTimeout(0.1)
        ),

        new ParallelCommandGroup(
          new RunPathPlannerTrajectory2(drivetrain, PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory4, DriverStation.getAlliance()), isFirstPath),
          new armPartial(arm).withTimeout(0.3),
          new StopIntake(intake).withTimeout(0.1)
        ),

        new armLow(arm).withTimeout(0.5),
        new JustOuttake(intake).withTimeout(0.1),
        new ParallelCommandGroup(
          new armMid(arm)
        ).withTimeout(0.15),
        new StopIntake(intake).withTimeout(0.05)
      )
    );
  }


  public void getsize(){
    // for(int i = 0; i<40; i++){
    // //   System.out.println(autopaths.size());
  
    // }
  
  }

}
