/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//|**************************************************************|
//|				 					 							 |
//|				Cyberheart 2018 First Power Up					 |
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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team6009.robot.Robot.Step;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.*;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	String gameData;
	// Auto Modes Setup
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	final String leftSwitch = "left Switch";
	String autoSelected;
	SendableChooser<String> chooser;
	
	//auto cases
	public enum Step { Straight, TurnLeft, TurnRight, Left, Right, Done }
	public Step autoStep = Step.Straight;
	public long timerStart;

	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, gripper, elevator;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis;
	
	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	// Joystick Definitions
	Joystick driver;
	
	// Boolean for buttons
	boolean aButton, bButton, yButton, xButton, leftBumper, rightBumper, start, select, leftThumbPush, rightThumbPush;
	
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	Solenoid ultra_solenoid;
	
	// Encoders
	Encoder leftEncoder, rightEncoder;
	
	// Gyro
	ADXRS450_Gyro gyroscope;
	
	//PID Variables
	PIDController rotationPID;
	
	// CONSTANT VARIABLES FOR PID
	//double Kp = 0.075;
	double Kp = 0.03;
	double Ki = 0;
	//double Kd = 0.195;
	double Kd = 0.0075;
	
	/*double kp = 0.005;
	double ki = 0.0;
	double kd = 0.0075;*/
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		chooser = new SendableChooser<String>();
		chooser.addObject("left Switch", leftSwitch);
		SmartDashboard.putData("Auto choices", chooser);

		
		// Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		gripper = new Spark(4);
		elevator = new Spark(5);
		
		driver = new Joystick(0);
		
		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		// Inverts the right side of the drive train to account for the motors being physically flipped
		rightChassis.setInverted(true);
		
		// Defines our DifferentalDrive object with both sides of our drivetrain
		chassis = new DifferentialDrive(leftChassis, rightChassis);
		
		// Set up port for Ultrasonic Distance Sensor
		ultrasonic_yellow = new AnalogInput(0);
		ultrasonic_black = new AnalogInput(1);
		ultra_solenoid = new Solenoid(0);
		
		
		// Set up Encoder ports
		leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(2,3);
		
		//Gyroscope Setup
		gyroscope = new ADXRS450_Gyro();
		gyroscope.calibrate();
		
		rotationPID = new PIDController(Kp, Ki, Kd, gyroscope, gripper);
		rotationPID.setSetpoint(0);
		
		
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
		autoSelected = (String) chooser.getSelected();
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
		if(gameData.charAt(0) == 'L'){
		if (autoSelected.equalsIgnoreCase(leftSwitch)) {
// for LL or LR
			switch (autoStep) {
			case Straight:
				driveStraight(0, 0.4);
				
				if (distance > 100){
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.TurnLeft;
				}
				// Put custom auto code here
				break;
			case TurnLeft:
				autoStep = Step.TurnRight;
				break;
			case TurnRight:
				rotationPID.setEnabled(true);
				rotationPID.setSetpoint(90);
				leftChassis.set(rotationPID.get());
				rightChassis.set(-rotationPID.get());
				rotationPID.setEnabled(false);
				
				timerStart = System.currentTimeMillis();
				autoStep = Step.Left;
				break;
			case Left:
				autoStep = Step.Right;
				break;
			case Right:
				driveStraight(90, 0.4);
				
				autoStep = Step.Done;
				break;
			case Done:
				leftChassis.set(0);
				rightChassis.set(0);
				break;
			}
			//default:
				// Put default auto code here
				//break;
			}
		}
		else if(gameData.charAt(0) == 'R'){
			
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		ultra_solenoid.set(true);
		//leftChassis.set(0.1);
		//rightChassis.set(0.1);
		chassis.arcadeDrive(driver.getX(), -driver.getY());
		
		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		leftBumper = driver.getRawButton(5);
		rightBumper = driver.getRawButton(6);
		select = driver.getRawButton(7);
		start = driver.getRawButton(8);
		leftThumbPush = driver.getRawButton(9);
		rightThumbPush = driver.getRawButton(10);
		
		SmartDashboard.putNumber("Ultrasonic Yellow Distance (cm):", getUltrasonicYellowDistance());
		SmartDashboard.putNumber("Ultrasonic Black Distance (cm):", getUltrasonicBlackDistance());
		
		if(rotationPID.isEnabled()){
			rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));
		}
		
		if (bButton){
			rotationPID.setEnabled(false);
		} 
		else if (aButton) {
			rotationPID.setEnabled(true);
			/*rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));*/
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
		if (leftBumper) {
			Kp -= 0.005;
		} else if (rightBumper) {
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
	
	public void PIDDriveStraight() {
		rotationPID.setEnabled(true);
		rotationPID.setSetpoint(0);
		chassis.tankDrive(((0.5)+(rotationPID.get())), (-(0.5)+(rotationPID.get())));
		//rightChassis.set((0.5)+(rotationPID.get()));
		//leftChassis.set((0.5)-(rotationPID.get()));
		System.out.println("Yes");
	}
	
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
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
	
	// This is the function that allows the robot to drive straight no matter what
	// It automatically corrects itself and stays locked onto the set angle
	private void driveStraight(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle()%360.0;
		System.out.println("driveStraight");
		double error = heading - currentAngle;
		//rotationPID.setEnabled(true);
		//rotationPID.setSetpoint(0);
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
		leftChassis.set(leftSpeed);
		rightChassis.set(rightSpeed);
	}
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}
	public void driveStraighter() {
		
	}
}