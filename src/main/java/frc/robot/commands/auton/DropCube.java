package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.intake.JustOuttake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.TapAutoCube;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;


public class DropCube extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public DropCube (SwerveDrivetrain drivetrain, IntakeSubsystem intake, ArmSubsystem arm) {
    
        addCommands(
            new TapAutoCube(intake).withTimeout(0.05),
            new ZeroGyro(drivetrain).withTimeout(0.05),
            new armMid(arm).withTimeout(0.3),
            
            new JustOuttake(intake).withTimeout(0.25),
            // new JustIntakeSlow(intake).withTimeout(0.2),
            // new armLow(arm).withTimeout(0.1)   , 
            new StopIntake(intake).withTimeout(0.1)
        );
    }
    
}
