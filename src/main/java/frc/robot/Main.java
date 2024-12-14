// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;


//NEVER touch anything in here unless you know exactly what you are doing.
//this should never be modified unless you are calling up specific bools to get the status of the
//RobotBase framework.

public final class Main {
  private Main() {}

  public static void main(String... args) {

    //subclasses "Robot" under the RobotBase framework
    RobotBase.startRobot(Robot::new);
  }
}
