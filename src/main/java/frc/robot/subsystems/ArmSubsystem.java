package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.math.Conversions;


import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    Conversions conversions = new Conversions();

    private TalonFX motor = new TalonFX(Constants.ArmSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();
    boolean click = false;
    boolean zero = false;
    DigitalInput input = new DigitalInput(Constants.ArmSubsystem.DIO_PORT);

    private static final double kP = 0.61; // 0.84
    private static final double kI = 0.00025;
    private static final double kD = 0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 6000;    //4000   // 62_000.0
    private static final double kAcceleration = 4500;   //3000 // 44_000.0



    public ArmSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        
        //current limit burn out the motor at states
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        intakeMotorConfig.supplyCurrLimit = driveSupplyLimit;
        motor.configAllSettings(intakeMotorConfig);

        motor.setSelectedSensorPosition(0.0);
        
        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);

        motor.configMotionCruiseVelocity(kVelocity);
        motor.configMotionAcceleration(kAcceleration);


        //set motor to brake
        this.motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        // motor.setNeutralMode(NeutralMode.EEPROMSetting);
        dashboard();

    }

    public void armMid(){
        zero = false;
      motor.set(TalonFXControlMode.MotionMagic, Constants.ArmSubsystem.MID_HEIGHT);
    }

    public void armGround(){
        zero = true;
      motor.set(TalonFXControlMode.MotionMagic, Constants.ArmSubsystem.GROUND_HEIGHT);
      zero = false;
    }

    public void armPartial() {
        zero = false;
        motor.set(TalonFXControlMode.MotionMagic, Constants.ArmSubsystem.PARTIAL_HEIGHT);
    }

    public void manualArmUp(){
        zero = false;
        motor.set(ControlMode.PercentOutput, -Constants.ArmSubsystem.MANUAL_SPEED);
    }

    public void manualArmDown(){
        zero = true;
        motor.set(ControlMode.PercentOutput, Constants.ArmSubsystem.MANUAL_SPEED);
    }

    public void stop () {
        zero = false;
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    public void testSwitch(){
        click = !input.get();
        if(click == true && zero == true){
            resetMotors();
            // System.out.println("CLICKED");
            // motor.set(TalonFXControlMode.PercentOutput, 0);
            // motor.set(TalonFXControlMode.MotionMagic, -500);
            // stop();
        }
    }

    public void resetMotors () {
        motor.setSelectedSensorPosition(0.0);
        System.out.println("reset" + motor.getSelectedSensorPosition());
    }

    public void antiLukeFeature(){
        if(encoderCount>-20){
            stop();
        }
        else if(encoderCount<-35000){
            stop();
        }
    }

    public void findZero(){
        boolean bool = false;
        while (!bool){
            // System.out.println("False");
            click = !input.get();

            if(!click){
                motor.set(ControlMode.PercentOutput, 0.15);
            }
            else{
                bool = true;
                motor.setSelectedSensorPosition(0);
                stop();
            }
        }
    }


    public void periodic(){
        testSwitch();
        // antiLukeFeature();
        click = !input.get();
        dashboard();

    }




    
}