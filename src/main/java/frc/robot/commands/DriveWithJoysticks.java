// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
//import frc.robot.Constants.Constants.Swerve;
//import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

public class DriveWithJoysticks extends CommandBase {
  private final Swerve swerveSubsystem;
  private final Vision limelight;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier relative;
  private final DoubleSupplier maxSpeed;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Swerve swerveSubsystem, Vision limelight, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, DoubleSupplier maxSpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.relative = relative;
    this.maxSpeed = maxSpeed;
    this.xLimiter = new SlewRateLimiter(2.0);
    this.yLimiter = new SlewRateLimiter(2.0);
    this.turnLimiter = new SlewRateLimiter(2.0);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(relative.getAsBoolean()) {
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.maxSpeed,
      modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.maxSpeed,
      modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.maxAngularVelocity,
      limelight.getPoseRotation()));
    } else {
      swerveSubsystem.drive(new ChassisSpeeds(
        modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * Constants.Swerve.maxSpeed,
        modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * Constants.Swerve.maxSpeed,
        modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * Constants.Swerve.maxAngularVelocity));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter) {
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    if(Math.abs(value)*Constants.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND <= Constants.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND*0.01){
      value = 0.0;
    }

    return value;
  }
}