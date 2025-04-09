// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static DriveTrain driveTrain = new DriveTrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings


    // set the default command for drivetrain to teleop drive
    // Fill in the nulls with negative joystick inputs
    driveTrain.setDefaultCommand(DriveCommands.teleopDrive(null, null, null));

    configureBindings();
  }


  // Driver and operator controls go here
  private void configureBindings() {
  }

  



  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
