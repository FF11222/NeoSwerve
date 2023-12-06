package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveDriveCommand extends CommandBase {
    private final SwerveSubsystem subsystem;
    private final XboxController controller;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(3.0);

    public SwerveDriveCommand(SwerveSubsystem subsystem, XboxController controller) {
        this.subsystem = subsystem;
        this.controller = controller;

        addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = -MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.DEAD_BAND);
        double ySpeed = -MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.DEAD_BAND);
        double rotation = -MathUtil.applyDeadband(controller.getRightX(), DriveConstants.DEAD_BAND);

        this.subsystem.drive(
                this.xSpeedLimiter.calculate(xSpeed) * DriveConstants.MAX_SPEED,
                this.ySpeedLimiter.calculate(ySpeed) * DriveConstants.MAX_SPEED,
                this.rotateLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED,
                false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopModules();
    }
}
