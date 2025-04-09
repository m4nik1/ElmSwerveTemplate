// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DriveCommands {

    // Drive only in teleop
    public static Command teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        double speedMultipler = Constants.speedMultiTeleop;
        SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

        return Commands.run(() ->  {
            double getX = xSupplier.getAsDouble();
            double getY = ySupplier.getAsDouble();
            double getRotation = rotationSupplier.getAsDouble();

            double translateVal = translationLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getX, 01));
            double strafeVal = strafeLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getY, 01));
            double rotationVal = rotationLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getRotation, 01));

            Translation2d translation = new Translation2d(translateVal, strafeVal);

            RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);

        }
        , RobotContainer.driveTrain);
    }
}
