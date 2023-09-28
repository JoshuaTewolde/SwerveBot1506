package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.BackwardsSlow;
import frc.robot.commands.drivetrain.ForwardSlow2;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.Stop;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;


public class Center extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public Center (SwerveDrivetrain drivetrain, IntakeSubsystem intake,  ArmSubsystem arm, PathPlannerTrajectory trajectory1) {
        
        addCommands(
            new armMid(arm).withTimeout(0.2),
            new DropCube(drivetrain, intake, arm),
            new zeroArm(arm).withTimeout(1),
            new armMid(arm).withTimeout(0.3),
            new RunPathPlannerTrajectory2(drivetrain, trajectory1, true),
            new ForwardSlow2(drivetrain).until(() -> drivetrain.getGyroRoll() <11),
            new BackwardsSlow(drivetrain).withTimeout(1), //1.75 //1.25 for standish, ford field //1.1 for DTE field
            new Stop(drivetrain)
            //new InstantCommand(() -> drivetrain.
        );
    }
    
}
