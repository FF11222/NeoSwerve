package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
    private final XboxController driveController = new XboxController(0);
    private final SwerveDriveCommand driveCommand = new SwerveDriveCommand(driveSubsystem, driveController);
    
    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(this.driveCommand);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
