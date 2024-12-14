// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot_subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration;

/**Drivetrain subsystem class. Used to operate the Mecanum drive of the robot.*/
public class drivetrain extends SubsystemBase {

  //motor controller instances
  private final CANSparkMax FLmotor;
  private final CANSparkMax FRmotor;
  private final CANSparkMax RLmotor;
  private final CANSparkMax RRmotor;

  //gyro object
  private final Pigeon2 gyro;

  //kinematics object
  private final MecanumDrive drivesystem;

  //PID and FF controllers for heading
  private final ProfiledPIDController headingPID;
  private final SimpleMotorFeedforward headingFF;

  //convenience
  private static final double pi = Math.PI;

  //misc vars for storing desired and actual states
  private double Xspeed;
  private double Yspeed;
  private double Zspeed;
  private double headingdes;
  private double headingact;
  private double headingerror;
  private double headingPIDout;
  private double headingFFout;
  private double FLspeed;
  private double FRspeed;
  private double RLspeed;
  private double RRspeed;
  
  /**Drivetrain subsystem of the robot.*/
  public drivetrain() {
    
    //declare motor controllers
    FLmotor = new CANSparkMax(configuration.driveconfig.FLmotorID, MotorType.kBrushless);
    FRmotor = new CANSparkMax(configuration.driveconfig.FRmotorID, MotorType.kBrushless);
    RLmotor = new CANSparkMax(configuration.driveconfig.RLmotorID, MotorType.kBrushless);
    RRmotor = new CANSparkMax(configuration.driveconfig.RRmotorID, MotorType.kBrushless);

    //inversions for motors
    FRmotor.setInverted(true);
    RRmotor.setInverted(true);

    //gyroscope + zero
    gyro = new Pigeon2(configuration.driveconfig.gyroID);
    gyro.reset();

    //declare kinematics
    drivesystem = new MecanumDrive(FLmotor, RLmotor, FRmotor, RRmotor);

    //declare PID and FF controllers
    headingPID = new ProfiledPIDController(
      configuration.driveconfig.headingPIDkP,
      configuration.driveconfig.headingPIDkI,
      configuration.driveconfig.headingPIDkD, 
      configuration.driveconfig.headingconstraints);
    headingFF = new SimpleMotorFeedforward(
      configuration.driveconfig.headingFFkS,
      configuration.driveconfig.headingFFkV,
      configuration.driveconfig.headingFFkA);
    
    //make heading controller take continuous input
    headingPID.enableContinuousInput(-pi, pi);
  }

  /**Command used to operate mecanum drivetrain.
   * @param xSpeed Linear X speed. Must be in [-1, 1] interval with no units.
   * @param ySpeed Linear Y speed. Must be in [-1, 1] interval with no units.
   * @param zSpeed Rotational Z speed while in robot-centric. Must be in [-1, 1] interval with no units.
   * @param headingDes Desired heading in radians while using field-centric mode.
   * @param fieldOriented To operate in field-centric mode or robot-centric mode.
   */
  public Command drive(double xSpeed, double ySpeed, double zSpeed, double headingDes, boolean fieldOriented) {

    //set desired heading based on input heading value, calculate actual heading by getting gyro angle, converting to rads, 
    //and applying modulo to wrap, and calculate angle error + apply modulo to wrap for edge cases. Also pass linear speeds
    //out to vars so that they may be passed to telemetry manager
    headingact = MathUtil.angleModulus((gyro.getAngle()/180)*pi);
    headingdes = headingDes;
    headingerror = MathUtil.angleModulus(headingdes - headingact);
    Xspeed = xSpeed;
    Yspeed = ySpeed;
    FLspeed = FLmotor.get();
    FRspeed = FRmotor.get();
    RLspeed = RLmotor.get();
    RRspeed = RRmotor.get();

    //calculate PID and FF values
    headingPIDout = headingPID.calculate(headingact, headingdes);
    headingFFout = headingFF.calculate(headingerror);

    //conditional to switch between field oriented and robot oriented
    double zInput;
    if (fieldOriented) {
      zInput = headingPIDout + headingFFout;
    } else {
      zInput = Zspeed;
    }
    
    //return runnable that runs mecanum drive function
    return run(() -> drivesystem.driveCartesian(xSpeed, ySpeed, zInput, new Rotation2d(headingact)));
  }

  /**Zeros the gyroscope of the robot.*/
  public Command zeroGyro() {
    return run(() -> gyro.reset());
  }

  /**Returns an array of doubles containing pertinate drivetrain information.*/
  public double[] drivetrainData() {
    return new double[] {
      Xspeed, //0
      Yspeed, //1
      Zspeed, //2
      headingdes, //3
      headingact, //4
      headingerror, //5
      headingPIDout, //6
      headingFFout, //7
      FLspeed, //8
      FRspeed, //9
      RLspeed, //10
      RRspeed //11
    };
  }

  //this override will be called continuously every 20ms while the subsystem instance is active
  @Override
  public void periodic() {}
}
