package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.math.Conversions;


import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    Conversions conversions = new Conversions();

    private TalonFX motor = new TalonFX(Constants.IntakeSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();


    public IntakeSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        
        //current limit burn out the motor at states
        TalonFXConfiguration intakeMotorConfig   = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        intakeMotorConfig.supplyCurrLimit = driveSupplyLimit;
        motor.configAllSettings(intakeMotorConfig);


        //set motor to brake
        this.motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        dashboard();
    }

    public void intake (double x) {
        //previously x was defaultSpeed
        motor.set(TalonFXControlMode.PercentOutput, x);
        // System.out.println("Encoder: " + encoderCount);
    }

    public void intakeDefSpeed(){
        motor.set(TalonFXControlMode.PercentOutput, Constants.IntakeSubsystem.CUBE_DEFAULT_INTAKE_SPEED);

    }

    public void intakeSlowSpeed(){
        motor.set(TalonFXControlMode.PercentOutput, -0.2);
    }

    public void tapAutoCube(){ //to not drop the cone at start of auto
        motor.set(TalonFXControlMode.PercentOutput, 0.1);
    }


    public void outtakeDefSpeed(){
        motor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeSubsystem.CUBE_DEFAULT_OUTTAKE_SPEED);
    }

    public void outtakeAuto(){
        motor.set(TalonFXControlMode.PercentOutput, -0.5);
    }


    public void outtake () {
        motor.set(TalonFXControlMode.PercentOutput, -0.2); //-0.45
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }   

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    
}