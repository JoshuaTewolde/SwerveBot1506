// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class dropArmSpecific2 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem arm;
  private final SwerveDrivetrain drivetrain;
  private boolean finished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public dropArmSpecific2(ArmSubsystem arm, SwerveDrivetrain drivetrain) {
    this.arm = arm;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue)
      if(Math.abs(drivetrain.getGyroAngleDegrees()) > 30 && Math.abs(drivetrain.getGyroAngleDegrees()) < 175){
        arm.armGround();
      }
    else{
      if(Math.abs(drivetrain.getGyroAngleDegrees()) < 300 && Math.abs(drivetrain.getGyroAngleDegrees()) > 175){
        arm.armGround();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
