package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;

import static edu.wpi.first.math.MathUtil.applyDeadband;


public class AutoTrackCommand extends CommandBase {
    private final SwerveSubsystem subsystem;
    private final PIDController verticalSpeedController = new PIDController(
            VisionConstants.AutoTrackVerticalP, VisionConstants.AutoTrackVerticalI, VisionConstants.AutoTrackVerticalD);
    private final PIDController rotationSpeedController = new PIDController(
            VisionConstants.AutoTrackRotationP, VisionConstants.AutoTrackRotationI, VisionConstants.AutoTrackRotationD);
    public AutoTrackCommand(SwerveSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double aprilTagID = VisionManager.getAprilTagID();
        double distanceToGoal = VisionManager.getDistanceToGoalMeters();
        double distanceToGoalHorizontal = VisionManager.getHorizontalDistanceToGoalMeters(distanceToGoal);

        double limitDistance = 1.0;
        double verticalSpeed = this.verticalSpeedController.calculate(limitDistance, distanceToGoal);
        double rotationSpeed = -applyDeadband(this.rotationSpeedController.calculate(0, distanceToGoalHorizontal), 0.005);

        if (verticalSpeed > 0 && aprilTagID != -1) {
            this.subsystem.drive(verticalSpeed, 0.0, rotationSpeed, false);
        } else {
            this.subsystem.drive(0.0, 0.0, rotationSpeed, false);
        }

        SmartDashboard.putNumber("Target ID", aprilTagID);
        SmartDashboard.putNumber("Target Distance", distanceToGoal);
        SmartDashboard.putNumber("Target Horizontal Distance", distanceToGoalHorizontal);
        SmartDashboard.putNumber("Target Vertical Speed", verticalSpeed);
        SmartDashboard.putNumber("Target Rotation Speed", rotationSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.stopModules();
    }
}
