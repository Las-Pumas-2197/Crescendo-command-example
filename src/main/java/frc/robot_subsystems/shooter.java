// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot_subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private double leftPIDout;
  private double leftFFout;
  private double rightspeedact;
  private double rightspeeddes;
  private double rightspeederror;
  private double rightPIDout;
  private double rightFFout;
  private boolean atSetpoint;

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

  /**Runs the shooter on robot at a specified RPM.
   * @param velocityRPM RPM desired.
   */
  public Command shooterrun(double velocityRPM) {

    //write to internal values, speed actuals are gathered from encoders, speed des is the setpoint passed into this command,
    //error is the difference between the two
    leftspeedact = leftshooterencoder.getVelocity();
    leftspeeddes = velocityRPM;
    leftspeederror = leftspeeddes - leftspeedact;
    rightspeedact = rightshooterencoder.getVelocity();
    rightspeeddes = velocityRPM;
    rightspeederror = rightspeeddes - rightspeedact;
    
    //calculatte PIDs and FFs
    leftPIDout = leftPID.calculate(leftshooterencoder.getVelocity(), velocityRPM);
    leftFFout = leftFF.calculate(velocityRPM);
    rightPIDout = rightPID.calculate(leftshooterencoder.getVelocity(), velocityRPM);
    rightFFout = rightFF.calculate(velocityRPM);

    //run two runnables in parallel for left and right shooter and write PIDF outputs to motors
    //parallel command will not finish until both subsequent runnables are finished or until interrupted
    return parallel(
      run(() -> leftshooter.setVoltage(leftPIDout + leftFFout)).finallyDo(() -> leftshooter.setVoltage(0)),
      run(() -> rightshooter.setVoltage(rightPIDout + rightFFout)).finallyDo(() -> rightshooter.setVoltage(0))
    );
  }

  /**Returns a bool that explains whether both PID controllers are at setpoint.*/
  public boolean atSetpoint() {
    
    //check if PIDs are within tolerance, if yes then atSetpoint = true
    if (leftPID.atSetpoint() & rightPID.atSetpoint()) {
      atSetpoint = true;
    } else {
      atSetpoint = false;
    }
    return atSetpoint;
  }

  /**Runs an ejection routine to remove the stored note from the robot controllably.*/
  public Command shootereject() {

    //run shooter motors at 1/4 max speed to controllably eject note from the robot
    return parallel(
      run(() -> leftshooter.set(0.25)).finallyDo(() -> leftshooter.setVoltage(0)),
      run(() -> rightshooter.set(0.25)).finallyDo(() -> rightshooter.setVoltage(0)));
  }

  /**Runs intake continuously until note is detected or command is ended.*/
  public Command intake() {

    //run intakes until note sensor changes state to true. note that .until decorator only accepts booleansupplier.
    //any "Supplier" subtype can be generated using the :: function on a function that returns a normal var,
    //such as a long, int, double, or boolean
    return parallel(
      run(() -> lowerintake.set(1)).until(notesensor::get).finallyDo(() -> lowerintake.setVoltage(0)),
      run(() -> upperintake.set(1)).until(notesensor::get).finallyDo(() -> upperintake.setVoltage(0)));
  }

  /**Runs the motors in reverse at 1/4 speed to remove any jam in intake system.*/
  public Command intakeeject() {
    return parallel(
      run(() -> lowerintake.set(-0.25)).finallyDo(() -> lowerintake.setVoltage(0)),
      run(() -> upperintake.set(-0.25)).finallyDo(() -> upperintake.setVoltage(0)));
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
      leftPIDout, //6
      leftFFout, //7
      rightPIDout, //8
      rightFFout //9
    };
  }

  @Override
  public void periodic() {}
}
