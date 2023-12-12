package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class SwerveModule implements IDashboardProvider {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;
    private final PIDController turningPIDController;
    private final double absoluteEncoderOffset;
    private double driveOutput;
    private double turnOutput;
    private final String name;

    public SwerveModule(int driveMotorPort, int turningMotorPort, int turningEncoderPort, double absoluteEncoderOffset
            , boolean driveMotorReversed, boolean turningMotorReversed, String name) {
        this.name = name;
        this.registerDashboard();

        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorPort, MotorType.kBrushless);

        this.driveMotor.restoreFactoryDefaults();
        this.turningMotor.restoreFactoryDefaults();

        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.turningMotor.setIdleMode(IdleMode.kBrake);

        this.driveMotor.setInverted(driveMotorReversed);
        this.turningMotor.setInverted(turningMotorReversed);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turningEncoder = new CANCoder(turningEncoderPort);

        this.turningEncoder.configFactoryDefault();

        this.driveEncoder.setPositionConversionFactor(2.0 * Math.PI * RobotConstants.WHEEL_RADIUS / RobotConstants.GEAR_RATIO);
        this.driveEncoder.setVelocityConversionFactor(2.0 * Math.PI * RobotConstants.WHEEL_RADIUS / RobotConstants.GEAR_RATIO / 60.0);

        this.turningPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        this.absoluteEncoderOffset = absoluteEncoderOffset;
    }

    public double getTurningPosition() {
        return this.turningEncoder.getAbsolutePosition() - this.absoluteEncoderOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromDegrees(this.getTurningPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromDegrees(this.getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, this.getState().angle);

        this.driveOutput = state.speedMetersPerSecond / RobotConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        this.turnOutput = this.turningPIDController.calculate(this.getState().angle.getRadians(), state.angle.getRadians());

        this.driveMotor.set(this.driveOutput);
        this.turningMotor.set(this.turnOutput);
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.name + " DrivePosition", this.driveEncoder.getPosition());
        SmartDashboard.putNumber(this.name + " DriveVelocity", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + " TurnPosition", this.getTurningPosition());
        SmartDashboard.putNumber(this.name + " TurnVelocity", this.turningEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + " DriveSpeed", this.driveOutput);
        SmartDashboard.putNumber(this.name + " TurnSpeed", this.turnOutput);
    }
}
