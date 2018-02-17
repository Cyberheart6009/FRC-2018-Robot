/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//|**************************************************************|
//|				Cyberheart 2018 First Power Up 					 |
//|																 |
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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.*;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	
	// Auto Modes Setup
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	
	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	
	// Smartdashboard Chooser object for Auto modes
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//defining limit switches
	DigitalInput limitSwitchUp, limitSwitchDown, limitSwitchSide;
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, climberOne, climberTwo, elevatorOne, elevatorTwo;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, climberGroup, elevatorGroup;
	
	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	// Joystick Definitions
	Joystick driver, operator;
	
	//Boolean for buttons
	boolean aButton, bButton;
	
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	Solenoid ultra_solenoid;
	
	// Encoders
	Encoder leftEncoder, rightEncoder;
	
	// Gyro
	ADXRS450_Gyro gyroscope;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		
		// Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		
		climberOne = new Spark(4);
		climberTwo = new Spark(5);

		elevatorOne = new Spark(6);
		elevatorTwo = new Spark(7);
				
		limitSwitchUp = new DigitalInput(4);
		limitSwitchDown = new DigitalInput(5);
		limitSwitchSide = new DigitalInput(6);
		
		driver = new Joystick(0);
		operator = new Joystick(1);
		
		
		
		climberGroup = new SpeedControllerGroup(climberOne, climberTwo);
		elevatorGroup = new SpeedControllerGroup(elevatorOne, elevatorTwo);
		
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
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
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
		/*
		if (bButton){
			gripper.set(-0.4);
		} 
		else if (aButton) {
			gripper.set(0.4);
		} 
		else {
			gripper.set(0);
		}*/

		updateSmartDashboard();
	
		elevatorGroup.set(driver.getRawAxis(5));
		climberGroup.set(driver.getRawAxis(5));
		climberOne.setInverted(true);
		
	    if (!limitSwitchUp.get()) {
			if (driver.getRawAxis(5) < 0) {
				elevatorGroup.set(0);
				climberGroup.set(0);
			}
			else {
				elevatorGroup.set(driver.getRawAxis(5));
				climberGroup.set(driver.getRawAxis(5));
			}
		}
		
		if (!limitSwitchDown.get()) {
			if (driver.getRawAxis(5) > 0) {
				elevatorGroup.set(0);
				climberGroup.set(0);
			}
			
			else {
				elevatorGroup.set(driver.getRawAxis(5));
				climberGroup.set(driver.getRawAxis(5));
			}
		}
		/*
		if (!limitSwitchSide.get()) {
			if (driver.getRawAxis(4) < 0) {
				gripper.set(0);
			}
			
			else {
				gripper.set(driver.getRawAxis(4));
			}
		}
		*/
	
		
		if (!limitSwitchUp.get()) {
			System.out.println("Up Activated");
		}
		if (!limitSwitchDown.get()) {
			System.out.println("down activated");
		}
		
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void disabledPeriodic(){
		updateSmartDashboard();
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

 	public void updateSmartDashboard() { 
 		 
 		//SmartDashboard.putData("Gyro:", gyroscope); 
 		//SmartDashboard.putNumber("Gyro Angle:", gyroscope.getAngle()); 
 		//SmartDashboard.putNumber("Gyro Rate:", gyroscope.getRate()); 
 		SmartDashboard.putNumber("Ultrasonic Yellow Distance (cm):", getUltrasonicYellowDistance()); 
 		SmartDashboard.putNumber("Ultrasonic Black Distance (cm):", getUltrasonicBlackDistance()); 		
 		
 		SmartDashboard.putBoolean("Limit Switch Up:", limitSwitchUp.get());
 		SmartDashboard.putBoolean("Limit Switch Down:", limitSwitchDown.get());
 		SmartDashboard.putBoolean("Limit Switch Side:", limitSwitchSide.get());
 		
		SmartDashboard.putNumber("Left Encoder Count:", leftEncoder.get()); 
 		SmartDashboard.putNumber("Right Encoder Count:", rightEncoder.get()); 
 		SmartDashboard.putNumber("Encoder Distance:", getDistance()); 
 	} 
 } 




