package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    private final TalonFX kRotationMotor = new TalonFX(0);
    private final TalonFX kFlyWheelMotor = new TalonFX(1);
    private final TalonFX kHoodMotor = new TalonFX(2);

    private final Pigeon2 pigeon;

    public enum Targets {
            NONE(Translation3d.kZero),
            HUB(new Translation3d(4.5, 8.1 / 2, 1.88)),
            NEUTRALZONE_FEED_LEFT(new Translation3d(2.3, 8.1 - 3.2, 0)),
            NEUTRALZONE_FEED_RIGHT(new Translation3d(2.3, 3.2, 0)),
            OPPOSING_ALLIANCE_FEED_LEFT(new Translation3d(7, 8.1 - 3.2, 0)),
            OPPOSING_ALLIANCE_FEED_RIGHT(new Translation3d(7.3, 3.2, 0)),
            OVERRIDE_BEHIND_HUB(new Translation3d(-4.64, 8.1 / 2, 0));

            Targets(Translation3d translation3d) {
                            this.position = translation3d;
                }
            
            public final Translation3d position;
        }



    public Turret(Pigeon2 pigeon) {
        this.pigeon = pigeon;

        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        
        rotationConfig.Slot0.kP = 0.1;
        rotationConfig.Slot0.kI = 0.0;
        rotationConfig.Slot0.kD = 0.0;
        
        rotationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        rotationConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        kRotationMotor.getConfigurator().apply(rotationConfig);

        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = 0.1;
        flywheelConfig.Slot0.kI = 0.0;
        flywheelConfig.Slot0.kD = 0.0;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        kFlyWheelMotor.getConfigurator().apply(flywheelConfig);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        hoodConfig.Slot0.kP = 0.1;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;

        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        kHoodMotor.getConfigurator().apply(hoodConfig);
        
    }

    

        
}
