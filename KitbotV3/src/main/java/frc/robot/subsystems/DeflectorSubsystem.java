package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DeflectorSubsystem extends SubsystemBase {

    private TalonFX deflectorMotor= new TalonFX(51);
    private TalonFXConfiguration deflectorConfiguration = new TalonFXConfiguration();

    private double intakeSpeed = -0.9;                                                   //CHECK
    private double shooterSpeed = 0.9;                                                 //CHECK


    public DeflectorSubsystem(){
        deflectorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        deflectorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;        
        //deflectorConfiguration.withCurrentLimits(
            //new CurrentLimitsConfigs()
            //.withStatorCurrentLimit(120)                          //CHECK
            //.withStatorCurrentLimitEnable(true));   

        deflectorMotor.getConfigurator().apply(deflectorConfiguration);
    }

    public void intakeDeflection(){
        deflectorMotor.set(intakeSpeed);
    }

    public void reverseIntakeDeflection(){
        deflectorMotor.set(-intakeSpeed);
    }

    public void shootDeflection(){
        deflectorMotor.set(shooterSpeed);
    }

    public void stop(){
        deflectorMotor.set(0);
    }
}
