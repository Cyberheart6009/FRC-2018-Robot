/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//|**************************************************************|
//|				Cyberheart 2018 First Power Up 					 |
//|					  Katarina Mavric							 |
//|																 |
//|**************************************************************|



package org.usfirst.frc.team6009.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// New Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;
//import roborio.java.src.com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.DigitalInput;

import com.kauailabs.navx.frc.AHRS;





/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends IterativeRobot implements PIDOutput {
	
	double rotateToAngleRate;
	
	// Constant defining encoder turns per inch of robot travel
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	
	// Auto Modes Setup
	String gameData;
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
    final String leftSwitch = "left Switch";
    final String leftSwitchSwitch = "left Switch Switch";
    final String rightSwitchSwitch = "right Switch Switch";
    final String square = "square";
	String autoSelected;
	SendableChooser<String> chooser;
	
	//auto cases
		public enum Step { Straight, Turn, Straight2, Turn2, Straight3, CubeOut, CubeIn, CubeOut2,  Done }
		public Step autoStep = Step.Straight;
		public long timerStart;
		
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	Solenoid ultra_solenoid;
	
	//Limit Switches
	DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber, limitSwitchGripper;

    SpeedController leftFront, leftBack, rightFront, rightBack, gripper1, gripper2, elevator1, elevator2, climber1, climber2, PIDSpeed;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis;
	
	Encoder leftEncoder, rightEncoder, leftElevator, rightElevator;
	
	DifferentialDrive chassis;
	
	Joystick driver; 
	
	//gyroscope
	AHRS gyroscope;
	
	boolean aButton, bButton, xButton, yButton, startButton, selectButton, upButton, downButton, lbumper, rbumper, start, select, leftThumbPush, rightThumbPush;
	
		//PID Variables
		PIDController rotationPID;
		
		// CONSTANT VARIABLES FOR PID
		//double Kp = 0.075;
		double Kp = 0.03;
		double Ki = 0;
		//double Kd = 0.195;
		double Kd = 0.0075;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		chooser.addDefault("Default Auto", kDefaultAuto);
		chooser.addObject("My Auto", kCustomAuto);
		
		chooser = new SendableChooser<String>();
		chooser.addObject("left Switch", leftSwitch);
		chooser.addObject("left Switch Switch", leftSwitchSwitch);
		chooser.addObject("right Switch Switch", rightSwitchSwitch);
		chooser.addObject("square", square);
		SmartDashboard.putData("Auto choices", chooser);
		
		limitSwitchUpElevator = new DigitalInput(4);
		limitSwitchDownElevator =  new DigitalInput(5);
		limitSwitchUpClimber = new DigitalInput(6);
		limitSwitchDownClimber = new DigitalInput(7);
		limitSwitchGripper = new DigitalInput(8);

		elevator1 = new Spark(6);
		elevator2 = new Spark(7);
		climber1 = new Spark(4);
		climber2 = new Spark(5);
		gripper1 = new Spark(8);
		gripper2 = new Spark(9);
		
		//leftElevator = new Encoder(4, 5);
		//rightElevator = new Encoder(6, 7);
		
		elevator2.setInverted(true);
		climber2.setInverted(true);
		gripper2.setInverted(true);
		
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		
		leftEncoder = new Encoder(0, 1);
		rightEncoder = new Encoder(2, 3);
		
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		
		rightChassis.setInverted(true);
		
		driver = new Joystick(0);
		
		chassis = new DifferentialDrive(leftChassis, rightChassis);
		
		gyroscope = new AHRS(SPI.Port.kMXP);
		
		
		rotationPID = new PIDController(Kp, Ki, Kd, gyroscope, this);
		rotationPID.setSetpoint(0);
		
		// Set up port for Ultrasonic Distance Sensor
		ultrasonic_yellow = new AnalogInput(0);
		ultrasonic_black = new AnalogInput(1);
		ultra_solenoid = new Solenoid(0);
		
	}
	
	


	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = (String)chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		gyroscope.reset();  // Reset the gyro so the heading at the start of the match is 0
	}
	

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double distance = getDistance();
		double height = getElevatorheight();
		if (autoSelected.equalsIgnoreCase(square)) {
			switch (autoStep) {
				case Straight:
					if (getDistance() < 24) {
						driveStraight(0, 0.3);
						System.out.println("Going Straight");
					}
					else {
						stop();
						resetEncoders();
						autoStep = Step.Turn;
						System.out.println("Encoders Reset!");
					}
					break;
					/*rotationPID.setEnabled(false);
					gyroscope.reset();
					resetEncoders();
					
					driveStraight(0, 0.3);
					
					if (distance > 10) {
						stop();
					}*/
				case Turn:
					resetEncoders();
					gyroscope.reset();
					rotationPID.setSetpoint(90);
					rotationPID.setEnabled(true);
					leftChassis.set(rotationPID.get());
					rightChassis.set(-rotationPID.get());
					autoStep = Step.Straight;
					rotationPID.setEnabled(false);
					break;
					/*if(turnRight(90)){
						stop();
						resetEncoders();
						gyroscope.reset();
						autoStep = Step.Straight;
						System.out.println("Turned Right");
					}*/
				case Done:
					stop();
					break;
				}
		}
		if (autoSelected.equalsIgnoreCase(leftSwitchSwitch) || autoSelected.equalsIgnoreCase(rightSwitchSwitch)) {
			switch (autoStep) {
				case Straight:
					resetEncoders();
					driveStraight(0, 0.4);
					if (distance > 220) {
						stop();
					}
					timerStart = System.currentTimeMillis();
					autoStep = Step.Turn;
					break;
				case Turn: 
					if (autoSelected == leftSwitchSwitch){
						rotationPID.setSetpoint(90);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
			
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight2;
					}
					else {
						rotationPID.setSetpoint(-90);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
						
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight2;
					}
					break;
				case Straight2:
					if (autoSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						resetEncoders();
						driveStraight(90, 0.4);
						if (distance > 45.5) {
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.Turn2;
					}
					else {
						rotationPID.setEnabled(false);
						resetEncoders();
						driveStraight(-90, 0.4);
						if (distance > 45.5) {
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.Turn2;
					}
					break;
				case Turn2:
					
					if (autoSelected == rightSwitchSwitch) {
						rotationPID.setSetpoint(180);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
						
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight3;
					}
					else {
						rotationPID.setSetpoint(-180);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
						
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight3;
					}
					break;
				case Straight3:
					/* could use vision tracking of cube, at this point the cube should be about in front of and centered 
					in regards to our robot so using the tracking will make sure we are in line with the cube*/
					
					if (autoSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						resetEncoders();
						
						driveStraight(180, 0.4);
						if (distance > 11){
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.CubeOut;
					}
					else {
						rotationPID.setEnabled(false);
						resetEncoders();
							
						driveStraight(-180, 0.4);
						if (distance > 11){
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.CubeOut;	
					}
					break;
				case CubeOut:
					elevator1.set(0.3);
					elevator2.set(0.3);
					if (height > 40) {
						stop();
					}
					gripper1.set(0.3);	
					gripper2.set(0.3);
					if (!limitSwitchGripper.get()) {
						gripper1.set(0);
						gripper2.set(0);
					}
					timerStart = System.currentTimeMillis();
					autoStep = Step.CubeIn;
					break;
				case CubeIn:
					rotationPID.setEnabled(false);
					resetEncoders();
					if (autoSelected == leftSwitchSwitch) {
						elevator1.set(-0.3);
						elevator2.set(-0.3);
						if (limitSwitchDownElevator.get()) {
							stop();
						}
						driveStraight(180, 0.4);
						if (distance > 13) {
							stop();
						}
						gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.CubeOut2;
					}
					else {
						elevator1.set(-0.3);
						elevator2.set(-0.3);
						if (limitSwitchDownElevator.get()) {
							stop();
						}
						driveStraight(-180, 0.4);
						if (distance > 13) {
							stop();
						}
						gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.CubeOut2;
					}
					break;
				case CubeOut2:
					elevator1.set(0.3);
					elevator2.set(0.3);
					if (height > 40) {
						stop();
					}
					gripper1.set(0.3);
					gripper2.set(0.3);
					if (!limitSwitchGripper.get()) {
						gripper1.set(0);
						gripper2.set(0);
					}
					timerStart = System.currentTimeMillis();
					autoStep = Step.Done;
					break;
				
				case Done:
					stop();
					break;
			}
		}
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		ultra_solenoid.set(true);
		
		chassis.arcadeDrive(driver.getX(), -driver.getY());
		
		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		lbumper = driver.getRawButton(5);
		rbumper = driver.getRawButton(6);
		select = driver.getRawButton(7);
		start = driver.getRawButton(8);
		leftThumbPush = driver.getRawButton(9);
		rightThumbPush = driver.getRawButton(10);
		
		//boolean rotateToAngle = false;
		
		if(rotationPID.isEnabled()){
			rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));
		}
		
		if (bButton){
			//rotateToAngle = true;
			//rotationPID.setSetpoint(0);
					
			rotationPID.setEnabled(false);
		} 
		else if (aButton) {
			//rotateToAngle = true;
			//rotationPID.setSetpoint(180);
			rotationPID.setEnabled(true);
			rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));
		}
		if(xButton){
			gyroscope.reset();
			resetEncoders();
		}
		if(yButton){
			driveStraight(0, 0.4);
			//chassis.tankDrive(0.5, -0.5);
			//rotationPID.setEnabled(false);
			//System.out.println("yButton");
		}
		if (lbumper) {
			Kp -= 0.005;
		} else if (rbumper) {
			Kp += 0.005;
		}
		if (select) {
			Ki -= 0.005;
		} else if (start) {
			Ki += 0.005;
		}
		if (leftThumbPush) {
			Kd -= 0.005;
		} else if (rightThumbPush) {
			Kd += 0.005;
		}
		
		
       /* if ( rotateToAngle ) {
            rotationPID.enable();
            rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));
            
        } else {
            rotationPID.disable();
            
        }*/
		
		//System.out.println(rotationPID.get());
		updateSmartDashboard();
		 
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	public void disabledPeriodic(){
		//System.out.println(rotationPID);
		updateSmartDashboard();
	}
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
	}
	public double getElevatorheight(){
		return ((double)(leftElevator.get() + rightElevator.get()) / (ENCODER_COUNTS_PER_INCH * 2));
	}
	public double getDistance(){
		return ((double)(leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
	}
	
	public double getUltrasonicYellowDistance(){
		// Calculates distance in centimeters from ultrasonic distance sensor
		return (double)(((ultrasonic_yellow.getAverageVoltage()*1000)/238.095)+9.0); //accuracy of 2 millimeters ;)
	}

	public double getUltrasonicBlackDistance(){
		// Calculates distance in centimeters from ultrasonic distance sensor
		return (double)(((ultrasonic_black.getAverageVoltage()*1000)/9.4));
	}
	
	private void driveStraight(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle()%360.0;
		System.out.println("driveStraight");
		double error = heading - currentAngle;

		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;

		// FIXME: This code can make the robot turn very 
		//        quickly if it is not pointed close to the
		//        correct direction.  I bet this is the 
		//        problem you are experiencing.
		//        I think if you correct the state machine
		//        if statements above, you will be able to 
		//        control the turning speed.
		
		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motors
			leftSpeed += error * Kp;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * Kp;
		}
	
		// set the motors based on the inputted speed
		leftBack.set(leftSpeed);
		leftFront.set(leftSpeed);
		rightBack.set(rightSpeed);
		rightFront.set(rightSpeed);
	}
	
    private void updateSmartDashboard() {
		
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());

		SmartDashboard.putNumber("Left Encoder Count", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Count", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
		
		SmartDashboard.putNumber("Kp: Bumpers", Kp);
		SmartDashboard.putNumber("Ki: StartSelect", Ki);
		SmartDashboard.putNumber("Kd: JoyPress", Kd);
	}
    
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
		elevator1.set(0);
		elevator2.set(0);
	}
	
	@Override
	   /*This function is invoked periodically by the PID Controller, */
	  /* based upon navX-MXP yaw angle input and PID Coefficients.    */
	  public void pidWrite(double output) {
	      rotateToAngleRate = output;
	  }
	
}
/*case Straight2:
resetEncoders();
driveStraight(90, 0.3);
if (distance > 10) {
	stop();
}
timerStart = System.currentTimeMillis();
autoStep = Step.Turn2;
break;
case Turn2:
rotationPID.setEnabled(true);
rotationPID.setSetpoint(180);
leftChassis.set(rotationPID.get());
rightChassis.set(-rotationPID.get());
rotationPID.setEnabled(false);

timerStart = System.currentTimeMillis();
autoStep = Step.Straight3;
break;
case Straight3:
resetEncoders();
driveStraight(180, 0.3);
if (distance > 10) {
	stop();
}
timerStart = System.currentTimeMillis();
autoStep = Step.Turn3;
break;
case Turn3:
rotationPID.setEnabled(true);
rotationPID.setSetpoint(270);
leftChassis.set(rotationPID.get());
rightChassis.set(-rotationPID.get());
rotationPID.setEnabled(false);

timerStart = System.currentTimeMillis();
autoStep = Step.Straight4;
break;
case Straight4:
resetEncoders();
driveStraight(270, 0.3);
if (distance > 10) {
	stop();
}
timerStart = System.currentTimeMillis();
autoStep = Step.Turn4;
break;
case Turn4:
rotationPID.setEnabled(true);
rotationPID.setSetpoint(0);
leftChassis.set(rotationPID.get());
rightChassis.set(-rotationPID.get());
rotationPID.setEnabled(false);

timerStart = System.currentTimeMillis();
autoStep = Step.Done;
break;*/

