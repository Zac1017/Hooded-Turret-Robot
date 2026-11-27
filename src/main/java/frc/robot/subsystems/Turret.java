package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveDrive;

public class Turret extends SubsystemBase{

    private final TalonFX kRotationMotor = new TalonFX(0);
    private final TalonFX kFlyWheelMotor = new TalonFX(1);
    private final TalonFX kHoodMotor = new TalonFX(2);

    private final PositionDutyCycle kClosedLoopPosition = new PositionDutyCycle(0.0d).withEnableFOC(false);
    private final VelocityDutyCycle kClosedLoopVelocity = new VelocityDutyCycle(0.0d).withEnableFOC(false);

    private final VoltageOut kOpenLoop = new VoltageOut(0.0d);

    private final SwerveDrive swerve;

    private boolean hasZeroedPosition = false;
    private double zeroPosition;
    private double globalRotation;


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

        public Targets target = Targets.NONE;


    public Turret(SwerveDrive swerve) {
        this.swerve = swerve;

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

    public void setRotationPosition(double position) {
        kRotationMotor.setControl(kClosedLoopPosition.withPosition((position / 360) * 5)); //TODO: Find GEAR RATIO
    }

    public void setVelocity(double velocity) {
        kFlyWheelMotor.setControl(kClosedLoopVelocity.withVelocity(velocity));
    }

    public void setHoodPosition(double position) {
        kHoodMotor.setControl(kClosedLoopPosition.withPosition(position));
    }


    public Translation3d getRobotPose() {
        return new Translation3d(swerve.getRawOdometeryPose().getTranslation().getX(),
            swerve.getRawOdometeryPose().getTranslation().getY(),
             0.565
        );
    }

    public Rotation2d getRobotRotation() {
        return swerve.getRawOdometeryPose().getRotation();
    }

    public Rotation2d getTurretRotation() {
        double motorRotations = kRotationMotor.getPosition().getValueAsDouble();

        double turretRotations = motorRotations / 5; //TODO: FIND GEAR RATIO!!!!!

        return Rotation2d.fromRotations(turretRotations);
    }

    public Translation3d getTranslationFromTarget(Targets target) {
        Translation3d robotTranslation = getRobotPose();

        Translation3d targetTranslation = new Translation3d(target.position.getX(), 
            target.position.getY(), 
            target.position.getZ())
        ;

        return new Translation3d(
            targetTranslation.getX() - robotTranslation.getX(),
            targetTranslation.getY() - robotTranslation.getY(),
            targetTranslation.getZ() - robotTranslation.getZ()
        );
    }

    public Rotation2d getDesiredTurretAngle(Targets target) {

        Translation3d displacement = getTranslationFromTarget(target);
        
        Rotation2d targetAngle = new Rotation2d(
            displacement.getX(),
            displacement.getY()
        );

        return targetAngle.minus(getRobotRotation());

    }

    public Rotation2d getAngleToTarget(Targets target) {

        return getDesiredTurretAngle(target).minus(getTurretRotation());
    }

    public double[] getThetaAndVelocity(Targets target) {

        Translation3d displacement = getTranslationFromTarget(target);

        double deltaX = Math.hypot(
            displacement.getX(),
            displacement.getY()
        );

        double deltaY = displacement.getZ();

        if (deltaX <= 0 || !Double.isFinite(deltaX) || !Double.isFinite(deltaY)) { //isFinite checks if the double is a valid, finite number
            return new double[]{Double.NaN, Double.NaN};
        }

        double optimalTheta = 45.0 + 0.5 * Math.toDegrees(
            Math.atan2(deltaY, deltaX)
        );
        
        double thetaRadians = Math.toRadians(optimalTheta);

        double denom = Math.pow(Math.cos(thetaRadians), 2) 
            * (deltaX * Math.tan(thetaRadians) - deltaY);

        if (denom <= 0 || !Double.isFinite(denom)) {
            return new double[]{Double.NaN, Double.NaN};
        }

        double initialVelocity = Math.sqrt((9.81 / 2) * Math.pow(deltaX, 2) / denom);

        if (!Double.isFinite(initialVelocity)) {
            return new double[]{Double.NaN, Double.NaN};
        }

        return new double[]{optimalTheta, initialVelocity};

        // Translation3d displacement = getTranslationFromTarget(target);
        // double deltaX = Math.hypot(displacement.getX(), displacement.getY());
        // double deltaY = displacement.getZ();

        // // initialVelocity = sqrt((-4.9 * (deltaX)^2) / (cos^2(theta) * (-deltaX * tan(theta) + deltaY)))

        // // dvi/dtheta = 1/2(4.9(deltaX)^2/cos^(2theta)(deltaY-deltaXtan(theta)))^(-1/2) * (-4.9(deltaX)^2*(-2sin(theta)cos(theta))(deltaY - deltaXtan(theta)+cos^2(theta)(-deltaXsec^2(theta))))/(cos^2(theta)(deltaY-deltaXtan(theta)))^2
        // //(deltaY - deltaXtan(theta)+cos^2(theta)(-deltaXsec^2(theta))))/(cos^2(theta)(deltaY-deltaXtan(theta)))^2

        // //minimum theta = 45 degrees + 1/2 tan^-1((deltaX / -deltaY))

        // double optimalTheta = 45.0 + 0.5 * Math.toDegrees(Math.atan2(deltaY, deltaX));

        // double initialVelocity = Math.sqrt(-4.9 * Math.pow(deltaX , 2) 
        // / (Math.pow(Math.cos(Math.toRadians(optimalTheta)), 2) * (-deltaX * Math.tan(Math.toRadians(optimalTheta)) 
        // + deltaY)));

        // if (deltaX <= 0 || !Double.isFinite(deltaX) || !Double.isFinite(deltaY)) {
        //     return new double[]{Double.NaN, Double.NaN};
        // }

        // return new double[]{optimalTheta, initialVelocity};
    }


    public void zeroTurret() {
        
            double current = Math.abs(kRotationMotor.getTorqueCurrent().getValueAsDouble());

            if (current > 0) { //TODO: Find Value for zeroe current threshold
                kRotationMotor.setControl(kOpenLoop.withOutput(0));
                kRotationMotor.setPosition(0);
                globalRotation = 0;
                hasZeroedPosition = true;
            } else {
                kRotationMotor.setControl(kOpenLoop.withOutput(1.5)); //TODO: Find Zeroing voltage
            }
    }

    public double projectileVelocityToMotorRPS(double velocity) {
        return 2 * (velocity / (2 * Math.PI * 1)); //TODO: FIND FLYWHEEL RADIUS IN METERS AND GEAR RATIO. 2 is gear ratio 1 is radius
    }

    @Override
    public void periodic() {
        if (!hasZeroedPosition) {
            zeroTurret();
            return;
        }

       setVelocity(projectileVelocityToMotorRPS(getThetaAndVelocity(target)[1]));
       setHoodPosition(getThetaAndVelocity(target)[0]);

       setRotationPosition(getAngleToTarget(target).getDegrees());
       
        // kFlyWheelMotor.setControl(kClosedLoopVelocity.withVelocity(getThetaAndVelocity(target)[1]));
        // kHoodMotor.setControl(kClosedLoopPosition.withPosition(getThetaAndVelocity(target)[0]));

        // kRotationMotor.setControl(kClosedLoopPosition.withPosition(getAngleToTarget(target).getDegrees()));

    }




        

        
}



