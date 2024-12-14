// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
//import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.configuration;

/**Shooter subsystem class. Use to operate the note shooter on the robot.*/
public class shooter extends SubsystemBase {

  //shooter + intake motors
  private final CANSparkMax upperintake;
  private final CANSparkMax lowerintake;
  private final CANSparkMax leftshooter;
  private final CANSparkMax rightshooter;

  //encoders for shooter
  private final RelativeEncoder leftshooterencoder;
  private final RelativeEncoder rightshooterencoder;

  //PID and FF controllers for shooter motors
  private final PIDController leftPID;
  private final PIDController rightPID;
  private final SimpleMotorFeedforward leftFF;
  private final SimpleMotorFeedforward rightFF;

  //note detector
  private final DigitalInput notesensor;

  //misc vars for internal data
  private double leftspeedact;
  private double leftspeeddes;
  private double leftspeederror;
  private double rightspeedact;
  private double rightspeeddes;
  private double rightspeederror;
  private double velocitysetpoint;
  private boolean atSetpoint;
  private boolean shooterready;

  public shooter() {

    //delcare motors
    upperintake = new CANSparkMax(configuration.shooterconfig.upperintakeID, MotorType.kBrushless);
    lowerintake = new CANSparkMax(configuration.shooterconfig.lowerintakeID, MotorType.kBrushless);
    leftshooter = new CANSparkMax(configuration.shooterconfig.leftshooterID, MotorType.kBrushless);
    rightshooter = new CANSparkMax(configuration.shooterconfig.rightshooterID, MotorType.kBrushless);


    //invert right shooter
    lowerintake.setInverted(true);
    rightshooter.setInverted(true);

    //encoders
    leftshooterencoder = leftshooter.getEncoder();
    rightshooterencoder = rightshooter.getEncoder();

    //PID and FF controllers for velocity
    leftPID = new PIDController(
      configuration.shooterconfig.shooterPIDkP,
      configuration.shooterconfig.shooterPIDkI,
      configuration.shooterconfig.shooterPIDkD);
    rightPID = new PIDController(
      configuration.shooterconfig.shooterPIDkP,
      configuration.shooterconfig.shooterPIDkI,
      configuration.shooterconfig.shooterPIDkD);
    leftFF = new SimpleMotorFeedforward(
      configuration.shooterconfig.shooterFFkS,
      configuration.shooterconfig.shooterFFkV,
      configuration.shooterconfig.shooterFFkA);
    rightFF = new SimpleMotorFeedforward(
      configuration.shooterconfig.shooterFFkS,
      configuration.shooterconfig.shooterFFkV,
      configuration.shooterconfig.shooterFFkA);

    //PID controller tolerances
    leftPID.setTolerance(configuration.shooterconfig.PIDtolerance);
    rightPID.setTolerance(configuration.shooterconfig.PIDtolerance);
    
    //DI for note sensor
    notesensor = new DigitalInput(configuration.shooterconfig.notesensorport);
  }

  /**Runs the shooter on robot at a specified RPM. Used exclusively for internal functions.
   * @param velocityRPM RPM desired.
   */
  private Command startshooterCL(double velocityRPM) {

    velocitysetpoint = velocityRPM;

    //run two runnables in parallel for left and right shooter and write PIDF outputs to motors
    //parallel command will not finish until both subsequent runnables are finished or until interrupted
    return parallel(

      run(() -> leftshooter.setVoltage(
        leftPID.calculate(leftshooterencoder.getVelocity(), velocityRPM) + leftFF.calculate(velocityRPM))),
        
      run(() -> rightshooter.setVoltage(
        rightPID.calculate(leftshooterencoder.getVelocity(), velocityRPM) + rightFF.calculate(velocityRPM))));
  }

  /**Stops the shooter motors. Used exclusively for internal functions. */
  private Command stopshooter() {

    //the .alongWith() function is a decorator similar in function to the parallel() method, just runs sequentially in 
    //line rather than in parallel on the same thread
    return runOnce(() -> leftshooter.set(0)).alongWith(runOnce(() -> rightshooter.set(0)));
  }

  /**Returns a bool that explains whether both PID controllers are at setpoint.*/
  public boolean atSetpoint() {
    
    //check if PIDs are within tolerance, if yes then atSetpoint = true
    atSetpoint = leftPID.atSetpoint() & rightPID.atSetpoint() ? true : false;

    return atSetpoint;
  }

  /**Runs an ejection routine to remove the stored note from the robot controllably.*/
  public Command shootereject() {

    //run shooter motors at 1/4 max speed to controllably eject note from the robot
    return parallel(
      run(() -> leftshooter.set(0.25)).finallyDo(() -> leftshooter.set(0)),
      run(() -> rightshooter.set(0.25)).finallyDo(() -> rightshooter.set(0)));
  }

  /**Ignores note sensor state and overrides intake motors to be on.*/
  public Command intakeoverride() {

    //runs intake motors at full speed, then sets speed to 0 when cancelled
    return parallel(
      run(() -> lowerintake.set(1)).finallyDo(() -> lowerintake.set(0)),
      run(() -> upperintake.set(1)).finallyDo(() -> upperintake.set(0)));
  }

  /**Runs the intake motors in reverse at 1/4 speed to help remove any jam in intake system.*/
  public Command intakeeject() {
    return parallel(
      run(() -> lowerintake.set(-0.25)).finallyDo(() -> lowerintake.set(0)),
      run(() -> upperintake.set(-0.25)).finallyDo(() -> upperintake.set(0)));
  }

  /**Runs intake continuously until note is detected or command is ended.*/
  public Command autointake() {

    //simply runs intake until note is detected. set(0) already handled by intakeoverride() function so not needed
    return run(() -> intakeoverride()).until(notesensor::get);
  }

  /**Spins up shooter to passed velocity, and feeds note into shooter.*/
  public Command shoot(double velocity) {

    //trigger goes true when at velocity setpoint, note is detected, and debounce period of 1 sec has passed. Then pass trigger
    //as boolean to shooterready var. This vastly simplifies and condenses shoot() logic
    shooterready = new Trigger(() -> atSetpoint).and(notesensor::get).debounce(1).getAsBoolean();

    //starts shooter at passed velocity, waits until shooterready boolean from trigger above passes true, then runs intake
    //until shooterready bool reads false (due to notesensor going false), then stops shooter and ends command
    return 
      run(() -> startshooterCL(velocity))
      .andThen(waitUntil(() -> shooterready)
      .andThen(intakeoverride())
      .until(() -> shooterready = false)
      .finallyDo(() -> stopshooter()));
  }

  /**Returns an array of doubles containing pertinate data from the shooter subsystem.*/
  public double[] shooterData() {
    return new double[] {
      leftspeedact, //0
      leftspeeddes, //1
      leftspeederror, //2
      rightspeedact, //3
      rightspeeddes, //4
      rightspeederror, //5
    };
  }

  @Override
  public void periodic() {
    //write to internal values, speed actuals are gathered from encoders, speed des is the setpoint passed into this command,
    //error is the difference between the two
    leftspeedact = leftshooterencoder.getVelocity();
    leftspeeddes = velocitysetpoint;
    leftspeederror = leftspeeddes - leftspeedact;
    rightspeedact = rightshooterencoder.getVelocity();
    rightspeeddes = velocitysetpoint;
    rightspeederror = rightspeeddes - rightspeedact;
  }
}
