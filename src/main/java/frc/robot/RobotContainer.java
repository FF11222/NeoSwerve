package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
    private final XboxController driveController = new XboxController(0);
    private final SwerveDriveCommand driveCommand = new SwerveDriveCommand(driveSubsystem, driveController);
    private final PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path"
            , Constants.AutoConstants.PHYSICAL_MAX_SPEED, Constants.AutoConstants.PHYSICAL_MAX_ACCELERATION);
    private final PIDController xPID = new PIDController(0.01, 0, 0);
    private final PIDController yPID = new PIDController(0.01, 0, 0);
    private final PIDController rotatePID = new PIDController(0.01, 0, 0);
    private final PPSwerveControllerCommand autoPathCommand = new PPSwerveControllerCommand(
            this.trajectory,
            this.driveSubsystem::getPose,
            this.xPID,
            this.yPID,
            this.rotatePID,
            this.driveSubsystem::setAutoModuleState,
            this.driveSubsystem
    );

    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(this.driveCommand);
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            this.autoPathCommand,
                Commands.run(this.driveSubsystem::stopModules, this.driveSubsystem)
        );
    }
}
