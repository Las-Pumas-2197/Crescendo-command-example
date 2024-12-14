// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot_utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot_subsystems.climbers;
import frc.robot_subsystems.drivetrain;
import frc.robot_subsystems.shooter;

/**Telemetry manager class.*/
public class telemetry_manager extends SubsystemBase {

  //instantiate subsystems
  private final drivetrain drivesubsystem;
  private final shooter shootersubsystem;
  private final climbers climbersubsystem;

  /**Telemetry manager class. Call up different functions to publish data to Shuffleboard.
   * @param drivesystem Drivetrain subsystem instance.
   */
  public telemetry_manager(drivetrain drivesystem, shooter shootersystem, climbers climbersystem) {

    //inject subsystems from robotcontainer to prevent multiple instances of the same objects
    drivesubsystem = drivesystem;
    shootersubsystem = shootersystem;
    climbersubsystem = climbersystem;

  }

  /**Publishes drivetrain data to Shuffleboard for analysis.*/
  public void drivetrainData() {

    //publish drivetrain data to Shuffleboard
    SmartDashboard.putNumber("Xspeed", drivesubsystem.drivetrainData()[0]);
    SmartDashboard.putNumber("Yspeed", drivesubsystem.drivetrainData()[1]);
    SmartDashboard.putNumber("Zspeed", drivesubsystem.drivetrainData()[2]);
    SmartDashboard.putNumber("Heading Desired", drivesubsystem.drivetrainData()[3]);
    SmartDashboard.putNumber("Heading Actual", drivesubsystem.drivetrainData()[4]);
    SmartDashboard.putNumber("Heading Error", drivesubsystem.drivetrainData()[5]);
    SmartDashboard.putNumber("Heading PID out", drivesubsystem.drivetrainData()[6]);
    SmartDashboard.putNumber("Heading FF out", drivesubsystem.drivetrainData()[7]);
    SmartDashboard.putNumber("FL motor speed", drivesubsystem.drivetrainData()[8]);
    SmartDashboard.putNumber("FR motor speed", drivesubsystem.drivetrainData()[9]);
    SmartDashboard.putNumber("RL motor speed", drivesubsystem.drivetrainData()[10]);
    SmartDashboard.putNumber("RR motor speed", drivesubsystem.drivetrainData()[11]);
  }

  /**Publishes shooter data to Shuffleboard for analysis.*/
  public void shooterData() {
    
    //publish shooter data to Shuffleboard
    SmartDashboard.putNumber("Shooter left speed actual", shootersubsystem.shooterData()[0]);
    SmartDashboard.putNumber("Shooter left speed desired", shootersubsystem.shooterData()[1]);
    SmartDashboard.putNumber("Shooter left speed error", shootersubsystem.shooterData()[2]);
    SmartDashboard.putNumber("Shooter right speed actual", shootersubsystem.shooterData()[3]);
    SmartDashboard.putNumber("Shooter right speed desired", shootersubsystem.shooterData()[4]);
    SmartDashboard.putNumber("Shooter right speed error", shootersubsystem.shooterData()[5]);
    SmartDashboard.putNumber("Shooter left PID output", shootersubsystem.shooterData()[6]);
    SmartDashboard.putNumber("Shooter left FF output", shootersubsystem.shooterData()[7]);
    SmartDashboard.putNumber("Shooter right PID output", shootersubsystem.shooterData()[8]);
    SmartDashboard.putNumber("Shooter right FF output", shootersubsystem.shooterData()[9]);
  }

public void climberData() {

  //publish climber data to Shuffleboard
  SmartDashboard.putNumber("Climber left pos actual", climbersubsystem.climberData()[0]);
  SmartDashboard.putNumber("Climber left pos des", climbersubsystem.climberData()[1]);
  SmartDashboard.putNumber("Climber right pos actual", climbersubsystem.climberData()[2]);
  SmartDashboard.putNumber("Climber right pos des", climbersubsystem.climberData()[3]);
  SmartDashboard.putBoolean("Climber left at setpoint", climbersubsystem.atSetpoint()[0]);
  SmartDashboard.putBoolean("Climber right at setpoint", climbersubsystem.atSetpoint()[1]);
}

  @Override
  public void periodic() {}
}
