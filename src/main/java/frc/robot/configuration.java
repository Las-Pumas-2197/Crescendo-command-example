// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**Configuration class which contains constants and configuration values for the robot.*/
public class configuration {

    /**Constants and configuration for the drivetrain of the robot. */
    public static class driveconfig {

        //for convenience
        private static final double pi = Math.PI;

        //motor controller IDs
        public static final int FLmotorID = 3;
        public static final int FRmotorID = 4;
        public static final int RLmotorID = 5;
        public static final int RRmotorID = 6;

        //gyroscope ID
        public static final int gyroID = 20;

        //PID and FF parameters, must be tuned
        public static final double headingPIDkP = 0.01; //proportional gain, direct multipler for controller output
        public static final double headingPIDkI = 0; //integral gain, affects steady-state error, try not to use
        public static final double headingPIDkD = 0; //derivative gain, affects rapid changes in setpoint
        public static final double headingFFkS = 0.1; //relates to static friction of system, in Volts
        public static final double headingFFkV = 1; //relationship between voltage and velocity, in Volts*rads/s
        public static final double headingFFkA = 0.25; //relationship between voltage and acceleration, in Volts*rads/s^2
        public static final double headingmaxvel = 2*pi; //rads/s
        public static final double headingmaxacl = 1*pi; //rads/s^2
        public static final TrapezoidProfile.Constraints headingconstraints = //constraints to limit output of PID controller
            new TrapezoidProfile.Constraints(
                headingmaxvel,
                headingmaxacl); 
    
    }

    /**Constants and configuration for the shooter of the robot.*/
    public static class shooterconfig {

        //motor IDs
        public static final int upperintakeID = 7;
        public static final int lowerintakeID = 8;
        public static final int leftshooterID = 9;
        public static final int rightshooterID = 10;

        //PID and FF paramters, must be tuned
        public static final double shooterPIDkP = 0.01;
        public static final double shooterPIDkI = 0;
        public static final double shooterPIDkD = 0;
        public static final double shooterFFkS = 0.1;
        public static final double shooterFFkV = 2;
        public static final double shooterFFkA = 0;        
        
        //PID tolerance
        public static final double PIDtolerance = 100; //RPM

        //note sensor port
        public static final int notesensorport = 0;
    }

    /**Constants and configuration for the climbers of the robot.*/
    public static class climberconstants {

        //motor IDs
        public static final int leftclimberID = 11;
        public static final int rightclimberID = 12;

        //encoder conversion factors
        public static final double encoderconvfactor = 1; //need calculated

        //motor ramp rate in units/sec, slew used to limit start current
        public static final double slewrate = 2; // unit/s, time to output calcs as 1/rate

        //digital input ports
        public static final int leftswport = 1;
        public static final int rightswport = 2;

        //setpoint height and tolerance
        public static final double setpointheight = 0.5; //meters, need measured
        public static final double setpointtol = 0.01; //roughly 3/8 of an inch
    }
}
