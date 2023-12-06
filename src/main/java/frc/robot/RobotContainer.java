// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.helpers.DashboardHelper;
import frc.lib.helpers.IDashboardProvider;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
    private final XboxController driveController = new XboxController(0);
    private final SwerveDriveCommand driveCommand = new SwerveDriveCommand(driveSubsystem, driveController);
    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(driveCommand);
        DashboardHelper.putAllRegistries();
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: Implement properly
        return null;
    }
}
