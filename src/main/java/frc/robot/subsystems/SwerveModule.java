package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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
    private final String name;

    public SwerveModule(int driveMotorPort, int turningMotorPort, int turningEncoderPort, double absoluteEncoderOffset
            , boolean driveMotorReversed, boolean turningMotorReversed, String name) {
        this.name = name;
        this.registerDashboard();

        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        this.turningMotor = new CANSparkMax(turningMotorPort, MotorType.kBrushless);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turningEncoder = new CANCoder(turningEncoderPort);

        this.turningPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        this.turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveMotor.setInverted(driveMotorReversed);
        this.turningMotor.setInverted(turningMotorReversed);

        this.driveEncoder.setPositionConversionFactor(2.0 * Math.PI * RobotConstants.GEAR_RATIO * RobotConstants.WHEEL_RADIUS);
        this.driveEncoder.setVelocityConversionFactor(2.0 * Math.PI * RobotConstants.GEAR_RATIO * RobotConstants.WHEEL_RADIUS / 60);

        this.absoluteEncoderOffset = absoluteEncoderOffset;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this.driveEncoder.getVelocity()
                , new Rotation2d(this.turningEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.driveEncoder.getPosition()
                , new Rotation2d(this.turningEncoder.getPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double driveOutput;
        double turningOutput;
        SwerveModuleState state;

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
        } else {
            state = SwerveModuleState.optimize(desiredState, this.getState().angle);

            driveOutput = state.speedMetersPerSecond / RobotConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
            turningOutput = this.turningPIDController.calculate(this.getState().angle.getRadians(), state.angle.getRadians());

            this.driveMotor.set(driveOutput);
            this.turningMotor.set(turningOutput);
        }
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.name + " DrivePosition", this.driveEncoder.getPosition());
        SmartDashboard.putNumber(this.name + " DriveVelocity", this.driveEncoder.getVelocity());
        SmartDashboard.putNumber(this.name + " TurnPosition", this.turningEncoder.getPosition());
        SmartDashboard.putNumber(this.name + " TurnVelocity", this.turningEncoder.getVelocity());

    }
}
