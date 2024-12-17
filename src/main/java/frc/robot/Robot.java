// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Example command robot to communicate proper command structure with tips + tricks.
//This is not a functional class, only to be used in simulation for testing or for teaching.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//subclass Robot that extends the RobotBase framework with TimedRobot framework, where multiple override methods may be 
//called for use during operation. TimedRobot is meant to be subclassed by the user that is developing the robot program, 
//so all instances for robot functions must be subclassed here.

//It is good practice to have most if not all code be subclassed through the robot container class in command structure,
//though certain commands and functions may be called in whatever override is needed if necessary.

public class Robot extends TimedRobot {

  //the actual robot object. All subsystems and commands should be subclassed through this object, which is declared in
  //the main robotcontainer class. Name can be changed to whatever is desired, as long as it is called up in the same way.
  private CrescendoBot system_CrescendoBot;

  //example autonomous command, can be any name or command. Instantiated up later in autonomousInit().
  private Command autoCommand;

  //An override that is called specifically when the robot code is initially started. It is called for
  //one scheduler cycle.
  @Override
  public void robotInit() {
    
    //declares the "robot container" object. Basically injects the robot object into this class, which can then be called 
    //upon by any override in this class.
    system_CrescendoBot = new CrescendoBot();
  }

  //Called continuously by the scheduler every 20ms, regardless of robot state. Things that should run continuously in 
  //the background while the robot is powered on should be called here.
  @Override
  public void robotPeriodic() {

    //Starts the command scheduler. The scheduler is used to queue commands and functions. It polls the states registered 
    //buttons, runs all queued commands, and ends all interrupted commands. The scheduler instance is a global 
    //singleton, meaning there must and will be only one instance through the entire Robot framework. As thus,
    //if the instance is never told to run with this line, no commands would be scheduled and no functions ran.
    //See https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html for more information.
    CommandScheduler.getInstance().run();
    system_CrescendoBot.runTelemetry();

  }

  //Called whenever the robot is disabled either through the driver's station or through the FMS at a competition.
  //This will be ran once by the scheduler when called.
  @Override
  public void disabledInit() {}

  //Called every 20ms by the scheduler when the robot is in the disabled state.
  @Override
  public void disabledPeriodic() {}

  //Called whenever the robot transitions from the disabled state to another state. Runs for one scheduler cycle.
  @Override
  public void disabledExit() {}

  //autonomousInit() is called once by the scheduler whenever it transitions to the autonomous state.
  @Override
  public void autonomousInit() {

    //instantiate autoCommand, can also be in robotInit
    autoCommand = system_CrescendoBot.getAutoCommand();

    //schedules command if command is not null to prevent crashes if command is null
    if (autoCommand != null) {
      autoCommand.schedule();
    }

    //call init commands during auto
    system_CrescendoBot.runInitCommands();
  }

  //Called continuously by the scheduler every 20ms whenever the robot is in the autonomous running state.
  @Override
  public void autonomousPeriodic() {}

  //Called whenever the robot exits the autonomous state.
  @Override
  public void autonomousExit() {}

  //Called whenever the robot enters the teleop state.
  @Override
  public void teleopInit() {
    
    //cancels auto command if not null to prevent crashes if command is null
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  //Called continuously by the scheduler every 20ms whenever the robot is in the teleop state.
  @Override
  public void teleopPeriodic() {

    //run teleop commands (including drive, shooter, and climber commands) during teleop
    system_CrescendoBot.runTeleopCommands();
  }

  //Called once by the scheduler whenever the robot exits teleop mode.
  @Override
  public void teleopExit() {}

  //Called once by the scheduler whenever robot transitions to test state.
  @Override
  public void testInit() {

    //Used to cancel all scheduled commands. If loop overruns occur due to bugs in robot code, or if too many commands are
    //scheduled at once and the scheduler is taking longer than 20ms for a cycle, use this to clean the scheduler in an
    //attempt to save the robot from a full crash of the code.
    CommandScheduler.getInstance().cancelAll();
  }

  //Called continuously by the scheduler every 20ms whenever the robot is in the test state.
  @Override
  public void testPeriodic() {}

  //called once by the scheduler whenever the robot transitions out of the test state.
  @Override
  public void testExit() {}
}
