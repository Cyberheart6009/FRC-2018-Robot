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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import org.spectrum3847.RIOdroid.RIOadb;
import org.spectrum3847.RIOdroid.RIOdroid;
import org.usfirst.frc.team6009.robot.Robot.Step;

import com.kauailabs.navx.frc.AHRS;

/**
	THIS BRANCH WAS CREATED TO JOIN CODE FROM BRANCHES JB, KM, RZ.
	THIS CODE IS FOR THE RYERSON DISTRICT EVENT
 */



public class Robot extends IterativeRobot {
	
	// Auto Modes Setup
	private static final String centerSwitch = "Center Switch";
	private static final String DriveStraight = "Straight Drive";
	private static final String RightSwitch = "Right Switch";
	private static final String LeftSwitch = "Left Switch";
	private String autoSelected;
	
	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
	final static double DEGREES_PER_PIXEL = 0.100;
	double currentSpeed;
	double oldEncoderCounts = 0;
	long old_time = 0;
	String box_position = "NO DATA";
	boolean initializeADB = false;
	String gameData;
	long timerStart;
	
	// Smartdashboard Chooser object for Auto modes
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, gripper, elevatorOne, elevatorTwo, climberOne, climberTwo, gripperOne, gripperTwo;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, climberGroup, gripperGroup, elevator;
	
	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	//LimitSwitch
	DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber, cubeSwitch;
	
	// Joystick Definitions
	Joystick driver;
	Joystick operator;
	
	//Boolean for buttons
	boolean aButton, bButton, xButton, yButton, aButtonOp, bButtonOp, xButtonOp, yButtonOp;
	
	// Encoders
	Encoder leftEncoder, rightEncoder, elevatorEncoder;
	
	// Gyro
	//ADXRS450_Gyro gyroscope;
	AHRS gyroscope;
	// PID Variables -WIP
	double kP = 0.03;
	
	//Auto variables
	public enum Step {STRAIGHT, SHORT_STRAIGHT, STRAIGHT1CENTER, TURN1CENTER, STRAIGHT2CENTER, TURN2CENTER, STRAIGHT3CENTER, TURN, LAUNCH, DONE};
	public Step autoStep = Step.STRAIGHT;
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		m_chooser.addObject("Center Switch", centerSwitch);
		m_chooser.addObject("My Auto", DriveStraight);
		m_chooser.addDefault("Right Switch", RightSwitch);
		m_chooser.addDefault("Left Switch", LeftSwitch);		
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
		gripperOne = new Spark(8);
		gripperTwo = new Spark(9);
		
		//Inverting Sparks
		gripperTwo.setInverted(true);
		climberOne.setInverted(true);
		
		// Defines Joystick ports
		driver = new Joystick(0);
		operator = new Joystick(1);
		
		//LimitSwitch Port Assignment
		cubeSwitch = new DigitalInput(4);

		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		climberGroup = new SpeedControllerGroup(climberOne, climberTwo);
		gripperGroup = new SpeedControllerGroup(gripperOne, gripperTwo);
		elevator = new SpeedControllerGroup(elevatorOne, elevatorTwo);
		
		// Inverts the right side of the drive train to account for the motors being physically flipped
		rightChassis.setInverted(true);
		
		// Defines our DifferentalDrive object with both sides of our drivetrain
		chassis = new DifferentialDrive(leftChassis, rightChassis);
		
		// Set up Encoder ports
		leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(3,2);
		elevatorEncoder = new Encoder(8,9);
		
		//Gyroscope (NAV-X) Setup
		gyroscope = new AHRS(SPI.Port.kMXP);
	
		// Initialize ADB Communication 
		if (initializeADB){
			RIOdroid.init();
			RIOadb.init();
			System.out.println("Start ADB" + RIOdroid.executeCommand("adb start-server"));
			System.out.println("ADB Initialization Complete");
		}
		else{
			System.out.println("ADB INIT NOT RAN");
		}
	}
	
	@Override
	public void autonomousInit() {
		autoSelected = (String) m_chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if ((autoSelected.equalsIgnoreCase(RightSwitch)) || autoSelected.equalsIgnoreCase(LeftSwitch)) {
			autoStep = Step.STRAIGHT;
		}
		
		resetEncoders();
		gyroscope.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		updateSmartDashboard();
		if (autoSelected == DriveStraight) {
			if (getDistance() < 160){
				driveStraight(0, 0.5);
				System.out.println("Going Straight");
			}
			else{
				stop();
				System.out.println("Stopped");
			}
		}
		
		if (autoSelected.equalsIgnoreCase(RightSwitch) || autoSelected.equalsIgnoreCase(LeftSwitch)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (getDistance() < 160){
					driveStraight(0, 0.5);
				}
				else{
					stop();
					autoStep = Step.TURN;
				}
				break;
			case TURN:
				if (gameData.charAt(0) == 'R' && autoSelected == RightSwitch){
					if (turnLeft(-90)) {
						resetEncoders();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else if(gameData.charAt(0) == 'L' && autoSelected == LeftSwitch){
					if (turnRight(90)) {
						resetEncoders();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else{
					stop();
					autoStep = Step.DONE;
				}
				break;
			case SHORT_STRAIGHT:
				if (autoSelected == LeftSwitch){
					if (getDistance() < 40){
						driveStraight(90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}
				else{
					if (getDistance() < 40){
						driveStraight(-90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}				
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 500) {
					gripperGroup.set(0.8);
				}
				else{
					gripperGroup.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		chassis.arcadeDrive(driver.getX(), -driver.getY());
		
		// Get Joystick Buttons
		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		
		aButtonOp = operator.getRawButton(1);
		bButtonOp = operator.getRawButton(2);
		xButtonOp = operator.getRawButton(3);
		yButtonOp = operator.getRawButton(4);

		
		/*		FOR DEBUGGING
		if (xButton) {
			//System.out.print(androidData());
			elevatorEncoder.reset();
			resetEncoders();
			gyroscope.reset();
		} 
		
		if (aButton){
			turnToBox();
		}*/
		
		// OPERATOR CONTROLS
		if (aButtonOp) {
			gripperGroup.set(1);
		}
		else if (yButtonOp) {
			gripperGroup.set(0.5);
		}
		else if (bButtonOp) {
			gripperGroup.set(-1);
		}
		else {
			gripperGroup.set(0);
		}
		climberGroup.set(operator.getRawAxis(5));
		elevator.set(operator.getRawAxis(1));

		updateSmartDashboard();
	}
				
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void disabledPeriodic(){
		updateSmartDashboard();
		autoSelected = (String) m_chooser.getSelected();
		SmartDashboard.putString("Selected Auto", autoSelected);
	}
	
	//--------------------------//
	//		Custom Functions	//
	//--------------------------//
	
	// Resets encoder values to 0
	public void resetEncoders(){
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	// Calculates and returns the Robot distance using the encoders attached to each side of the drive train
	public double getDistance(){
		return ((double)(leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
	}
	
	private double getElevatorHeight(){
		return (double)(elevatorEncoder.get()/ELEVATOR_ENCODER_COUNTS_PER_INCH);
	}
	
	// Calculates the robotSpeed
	public double robotSpeed() {
		// Calculates current speed of the robot in m/s
		currentSpeed = ((getDistance() - oldEncoderCounts)/(System.currentTimeMillis() - old_time)) * 0.0254;
		
		old_time = System.currentTimeMillis();
		oldEncoderCounts = getDistance();
		return (double) currentSpeed;
	}

	private double androidData(){
		double Center_X = 0;
		String box_data = RIOdroid.executeCommand("adb logcat -t 150 ActivityManager:I native:D *:S");
		String[] seperated_data = box_data.split("\n");
		//System.out.println(seperated_data[(seperated_data.length-1)]);
		// if split_data == 1 that means there was no line break & therefore no data
		if (seperated_data.length == 1){
			Center_X = 0;
		}
		else{	// if the length of the split array is longer than 1 then 
			box_position = seperated_data[(seperated_data.length-1)];
			String[] splitted;
	        double[] final_array = new double[4];
	        String[] split_data = box_position.split(",");
	        
	        if (split_data.length != 4){
	        	Center_X = 0;
	            //System.out.println("Invalid");
	        }
	        else{
	        	// this is not running
	            for (int i=0; i<split_data.length; i++){
	                if (i == 0){
	                    splitted = split_data[0].split(" ");
	                    final_array[0] = Double.parseDouble(splitted[splitted.length-1]);
	                }
	                else if(i == 3){
	                    splitted = split_data[3].split(" ");
	                    final_array[3] = Double.parseDouble(splitted[0]);
	                }
	                else{
	                    final_array[i] = Double.parseDouble(split_data[i]);
	                }
	            }
	            
	            /*	---		Uncomment to get the coordinates of the square tracking the cube --
	            for (int x=0; x<final_array.length; x++){
	                System.out.println(final_array[x]);
	            }*/
	            
	            Center_X = (final_array[3]+final_array[1])/2;
	        }
		}
		System.out.println("Center x: " + Center_X);
		return Center_X;
	}
	
	private void turnToBox(){
		double Center_X = androidData();
		//System.out.print("Offset: " + degreeOffset);
		
		// FIXME: Already removed the not before turnLeft/ turnRight
		if (Center_X > 170 && Center_X != 0){
			double degreeOffset = (170 - Center_X)*DEGREES_PER_PIXEL;
			double boxAngle = gyroscope.getAngle()%360 + degreeOffset;

			System.out.println("Should be turning left, Offset: " + degreeOffset);
			while (gyroscope.getAngle()%360 > boxAngle){
				turnLeft(boxAngle);
			}
		}
		if (Center_X < 170 && Center_X != 0){
			double degreeOffset = (170 - Center_X)*DEGREES_PER_PIXEL;
			double boxAngle = gyroscope.getAngle()%360 + degreeOffset;
			System.out.println("Should be turning right, Offset: " + degreeOffset);
			while(gyroscope.getAngle()%360 < boxAngle){
				turnRight(boxAngle);
			}
		}
		else{
			System.out.println("Should not be moving, NO DATA");
			stop();
		}
	}
	
	
	private void tipPrevention(){
		if (gyroscope.getPitch() > 10){
			leftChassis.set(0.5);
			rightChassis.set(0.5);
		}
		if (gyroscope.getPitch() < -10){
			leftChassis.set(-0.5);
			rightChassis.set(-0.5);
		}
	}
	
	private void driveStraight(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = (gyroscope.getAngle()%360.0);
		//System.out.println("driveStraight");
		double error = heading - currentAngle;
		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;
		
		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motors
			leftSpeed += error * kP;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * kP;
		}
	
		// set the motors based on the inputed speed
		leftChassis.set(leftSpeed);
		rightChassis.set(rightSpeed);
	}
	
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}
	
	//slow motor speeds while turning
	
	private boolean turnRight(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(0.3);
		leftFront.set(0.3);
		rightBack.set(0.3);
		rightFront.set(0.3);

		System.out.println("Turning Right");
		
		double currentAngle = gyroscope.getAngle();
		if (currentAngle >= targetAngle - 2){
			System.out.println("Stopped Turning Right");
			return true;
		}
		return false;
	}
	private boolean turnLeft(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(-0.3);
		leftFront.set(-0.3);
		rightBack.set(-0.3);
		rightFront.set(-0.3);

		double currentAngle = gyroscope.getAngle();
		if (currentAngle <= targetAngle + 2){
			return true;
		}
		return false;
	}
	// Pushes all data the Smart Dashboard when called
	private void updateSmartDashboard() {
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());

		SmartDashboard.putNumber("Left Encoder Count", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Count", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
		
		SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
		
		SmartDashboard.putBoolean("Cube Held", cubeSwitch.get());
		
		SmartDashboard.putNumber("Robot Speed", robotSpeed());
		
	}
}
