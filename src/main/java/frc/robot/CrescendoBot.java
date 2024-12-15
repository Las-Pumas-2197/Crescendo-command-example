// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.configuration.robotconfig;
import frc.robot_subsystems.climbers;
import frc.robot_subsystems.drivetrain;
import frc.robot_subsystems.shooter;
import frc.robot_utils.telemetry_manager;

public class CrescendoBot {

  //subsystem instances
  private final shooter subsys_shooter;
  private final drivetrain subsys_drivetrain;
  private final climbers subsys_climbers;
  private final telemetry_manager subsys_telemetry;

  //controller instances
  private final CommandJoystick joystick;
  private final CommandXboxController controller;

  //for convenience
  private static final double pi = Math.PI;

  //misc vars for joystick/controller transformations
  private double drivemult; //multiplier for drive speed
  private double headingdes; //desired heading to pass to drive subsystem
  private boolean fieldoriented; //bool to pass to drive subsystem to switch between field oriented and robot oriented control

  /**Main robot class. All subsystems and commands are subclassed here, so all functions must be called through this class.*/
  public CrescendoBot() {

    //declare subsystems, all subsystems must be passed through and injected into telemetry manager for telemetry to work
    subsys_climbers = new climbers();
    subsys_drivetrain = new drivetrain();
    subsys_shooter = new shooter();
    subsys_telemetry = new telemetry_manager(subsys_drivetrain, subsys_shooter, subsys_climbers);

    //declare controllers
    joystick = new CommandJoystick(robotconfig.joystickport);
    controller = new CommandXboxController(robotconfig.controllerport);

    //default commands for subsystems, currently only drive subsystem has a default command
    subsys_drivetrain.setDefaultCommand(new RunCommand(() -> 
      subsys_drivetrain.drive(
        MathUtil.applyDeadband(joystick.getX() * drivemult, robotconfig.joystickXYdeadband),
        MathUtil.applyDeadband(joystick.getY() * drivemult, robotconfig.joystickXYdeadband),
        MathUtil.applyDeadband(joystick.getZ() * drivemult, robotconfig.joystickZdeadband),
        headingdes,
        fieldoriented
    )));

  }

  /**Call this void to run all pertinate commands from button bindings during teleop.*/
  public void runTeleopCommands() {

    //vars for misc robot usage
    fieldoriented = joystick.trigger().getAsBoolean() ? false : true; //toggle field oriented using conditional
    drivemult = (joystick.getThrottle()+1)/2; //normalize throttle axis to [0, 1] interval and use as drive speed mult

    //joystick triggers. Unlike the normal Joystick object, CommandJoystick can automatically return a trigger instance to 
    //base an action off of.
    joystick.button(2).onTrue(subsys_drivetrain.zeroGyro()); //zero gyro when joystick side button is depressed

    //desired heading transformations
    joystick.povUp().onTrue(runOnce(() -> headingdes = 0));
    joystick.povDown().onTrue(runOnce(() -> headingdes = 1*pi));
    joystick.povLeft().onTrue(runOnce(() -> headingdes = 0.5*pi));
    joystick.povRight().onTrue(runOnce(() -> headingdes = -0.5*pi));

    //shooter operation bindings
    //if left bumper or right bumper is depressed, gives other layers of bindings

    //shooter eject layer, must use .whileTrue decorator or command will never end
    if (controller.leftBumper().getAsBoolean()) {
      controller.a().whileTrue(subsys_shooter.shootereject());
      controller.b().whileTrue(subsys_shooter.intakeeject());

    //climber override layer
    } else if (controller.rightBumper().getAsBoolean()) {
      controller.a().whileTrue(subsys_climbers.overrideright(-1));
      controller.y().whileTrue(subsys_climbers.overrideright(1));
      controller.povDown().whileTrue(subsys_climbers.overrideleft(-1));
      controller.povUp().whileTrue(subsys_climbers.overrideleft(1));

    //normal layer
    } else {
      controller.a().onTrue(subsys_shooter.shoot(2000)); //2000rpm, need measured for optimal performance
      controller.b().onTrue(subsys_shooter.autointake()); //automatically run intake until note is detected in pickup
      controller.povUp().onTrue(subsys_climbers.extend());
      controller.povDown().onTrue(subsys_climbers.retract());
    }
  }

  public void runTelemetry() {
    subsys_telemetry.climberData();
    subsys_telemetry.shooterData();
    subsys_telemetry.drivetrainData();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
