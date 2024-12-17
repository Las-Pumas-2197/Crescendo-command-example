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

  /**Call this void to run all commands that are needed to run at robot startup.*/
  public void runInitCommands() {

    //must be ran at start of robot operation to zero encoders. If climbers are already retracted far enough where they are
    //acutating the safety switches, then command will zero encoders and end. If not retracted fully, will retract climbers
    //until switch is depressed, then will zero encoders
    subsys_climbers.setZero();
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

    //shooter + climber operation bindings
    //if left bumper or right bumper is depressed, gives other layers of bindings

    //shooter eject layer, must use .whileTrue decorator or command will never end
    //added second condition to conditional to stop possibility of two commands being scheduled if both right and left bumper
    //are depressed and trigger button is depressed
    if (controller.leftBumper().getAsBoolean() & controller.rightBumper().getAsBoolean() != true) {
      controller.a().whileTrue(subsys_shooter.shootereject()); //eject note at low speed, runs shooters and intake
      controller.b().whileTrue(subsys_shooter.intakeeject()); //ejecte note backwards through intake

    //climber override layer
    //also added additional condition to prevent above problem
    } else if (controller.rightBumper().getAsBoolean() & controller.leftBumper().getAsBoolean() != true) {
      controller.a().whileTrue(subsys_climbers.overrideright(-1)); //override right down
      controller.y().whileTrue(subsys_climbers.overrideright(1)); //override right up
      controller.povDown().whileTrue(subsys_climbers.overrideleft(-1)); //override left down
      controller.povUp().whileTrue(subsys_climbers.overrideleft(1)); //override left up

    //normal layer
    } else {
      controller.a().onTrue(subsys_shooter.shoot(2000)); //2000rpm, need measured for optimal performance
      controller.b().onTrue(subsys_shooter.autointake()); //automatically run intake until note is detected in pickup
      controller.povUp().onTrue(subsys_climbers.extend()); //extend climbers automatically
      controller.povDown().onTrue(subsys_climbers.retract()); //retract climbers automatically
    }
  }

  /**Call this void to post telemetry to Shuffleboard.*/
  public void runTelemetry() {
    subsys_telemetry.climberData();
    subsys_telemetry.shooterData();
    subsys_telemetry.drivetrainData();
  }

  public Command getAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
}
