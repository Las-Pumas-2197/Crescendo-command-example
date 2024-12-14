// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot_subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration;

/**Subclass for climbers. Used to operate the climbers on the robot.*/
public class climbers extends SubsystemBase {

  //motors
  private final CANSparkMax leftmotor;
  private final CANSparkMax rightmotor;

  //encoders
  private final RelativeEncoder leftencoder;
  private final RelativeEncoder rightencoder;

  //digital inputs for 
  private final DigitalInput leftsw; //left climber retracted safety switch, NO
  private final DigitalInput rightsw; //right climber retracted safety switch, NO

  //tolerance and setpoint for convenience
  private static final double setpointtol = configuration.climberconstants.setpointtol;
  private static final double setpoint = configuration.climberconstants.setpointheight;

  //misc vars for class usage
  private double leftposact;
  private double leftposdes;
  private double rightposact;
  private double rightposdes;

  public climbers() {

    //declare motors
    leftmotor = new CANSparkMax(configuration.climberconstants.leftclimberID, MotorType.kBrushless);
    rightmotor = new CANSparkMax(configuration.climberconstants.rightclimberID, MotorType.kBrushless);

    //inversions, inversions must ALWAYS be before any dependent objects get declared, like encoders
    rightmotor.setInverted(true);

    //encoder objects
    leftencoder = leftmotor.getEncoder();
    rightencoder = rightmotor.getEncoder();

    //encoder conversion factors
    leftencoder.setPositionConversionFactor(configuration.climberconstants.encoderconvfactor);
    rightencoder.setPositionConversionFactor(configuration.climberconstants.encoderconvfactor);

    //declare digital inputs
    leftsw = new DigitalInput(configuration.climberconstants.leftswport);
    rightsw = new DigitalInput(configuration.climberconstants.rightswport);

  }

  //the method runEnd() for the below commands runs the first arg continuously with scheduler, the second arg is ran once 
  //when the command is interrupted. Works very similar to run() with the .finallyDo() decorator except this method does 
  //not cover edge cases where command finishes normally, but in this case this command should never end normally and 
  //should always be interrupted

  /**Runs the left climber without safeties using passed speed. Note that this may result in destruction of climber 
   * mechanism if mechanical limits are not respected.
   * @param speed The passed speed for the climbers. Interval is [-1, 1].
   */
  public Command overrideleft(double speed) {
    return runEnd(() -> leftmotor.set(speed), () -> leftmotor.set(0)); //run at "speed", set zero when interrupted
  }

  /**Runs the right climber without safeties using passed speed. Note that this may result in destruction of climber 
   * mechanism if mechanical limits are not respected.
   * @param speed The passed speed for the climbers. Interval is [-1, 1].
   */
  public Command overrideright(double speed) {
    return runEnd(() -> rightmotor.set(speed), () -> rightmotor.set(0));
  }

  
  /**Returns a bool array explaining whether climbers are at setpoint or not. Left = array ID#0, right = array ID#1.*/
  public boolean[] atSetpoint() {
    return new boolean[] {
      leftposdes - leftposact < setpointtol ? true : false, //0
      rightposdes - rightposact < setpointtol ? true : false //1
    };
  }

  /**Returns a double array containing pertinate climber position data.*/
  public double[] climberData() {
    return new double[] {
      leftposact,
      leftposdes,
      rightposact,
      rightposdes
    };
  }

  /**Commands climbers to retract until safety switch is triggered, and resets to encoder zero to that position + tolerance.*/
  public Command setZero() {
    
    //run climbers down in parallel until safety switch is depressed, then reset encoder to that position + tolerance
    return parallel(

      run(() -> overrideleft(-0.5)) //run climber
        .until(leftsw::get) //until switch returns true
        .finallyDo(() -> leftencoder.setPosition(setpointtol)), //set encoder to 0 + tolerance for space when retracting

      run(() -> overrideright(-0.5))
        .until(rightsw::get)
        .finallyDo(() -> rightencoder.setPosition(setpointtol)));
  }

  /**Runs climbers out to setpoint height, at which the command ends. If safety switch is depressed, cancel command.*/
  public Command extend() {

    //runs climbers out in parallel as long as safety switch is not depressed, until error is less than tolerance
    //note that the left switch logic is inverted due to how the .onlyWhile decorator functions
    return parallel(

      run(() -> overrideleft(1)) //run climber
        .onlyWhile(() -> leftsw.get() ? false : true) //run only if conditional returns true, otherwise cancel command
        .until(() -> leftposdes - leftposact < setpointtol ? true : false), //run until difference is less than tolerance

      run(() -> overrideright(1)) //same logic as above except on right climber
        .onlyWhile(() -> rightsw.get() ? false : true)
        .until(() -> rightposdes - rightposact < setpointtol ? true : false));
  }

  /**Runs climbers in to retracted height, at which command ends. If safety switch is depressed, cancel command.*/
  public Command retract() {

    //runs climbers in on a paralell in similar fashion to above, except it looks for pos act to be less than 0
    return parallel(

      run(() -> overrideleft(-1))
        .onlyWhile(() -> leftsw.get() ? false : true)
        .until(() -> leftposact < 0 ? true : false),

      run(() -> overrideright(-1))
        .onlyWhile(() -> rightsw.get() ? false : true)
        .until(() -> rightposact < 0 ? true : false));
  }

  @Override
  public void periodic() {

    //passing values for usage in commands above
    leftposact = leftencoder.getPosition();
    leftposdes = setpoint;
    rightposact = leftencoder.getPosition();
    rightposdes = setpoint;
  }
}
