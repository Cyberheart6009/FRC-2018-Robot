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
import org.spectrum3847.RIOdroid.RIOadb;
import org.spectrum3847.RIOdroid.RIOdroid;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

/*TODO: 	- Robot Tip prevention (if angle > threshold) -> take over drive train motors and reverse to prevent the tip
 * 			- Function to get phone logs and parse the data in order to retrieve and calculate the location of the box
 * 			
*/

public class Robot extends IterativeRobot {
	
	// Auto Modes Setup
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private static final String SquareAuto = "Square";
	private String m_autoSelected;
	
	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
	final static double DEGREES_PER_PIXEL_RIGHT = 0.07;
	final static double DEGREES_PER_PIXEL_LEFT = 0.100;
	double currentSpeed;
	double oldEncoderCounts = 0;
	long old_time = 0;
	String box_position = "NO DATA";
	boolean initializeADB = false;
	
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
	
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	
	// Encoders
	Encoder leftEncoder, rightEncoder, elevatorEncoder;
	
	// Gyro
	//ADXRS450_Gyro gyroscope;
	AHRS gyroscope;
	// PID Variables -WIP
	double kP = 0.03;
	
	//Auto variables
	public enum Step {STRAIGHT, TURN};
	public Step autoStep = Step.STRAIGHT;
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		m_chooser.addObject("Square Mode", SquareAuto);
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
		//elevatorOne.setInverted(true);
		gripperTwo.setInverted(true);
		
		// Defines Joystick ports
		driver = new Joystick(0);
		operator = new Joystick(1);
		
		//LimitSwitch Port Assignment
		limitSwitchUpElevator = new DigitalInput(4);
		limitSwitchDownElevator =  new DigitalInput(5);
		limitSwitchUpClimber = new DigitalInput(6);
		limitSwitchDownClimber = new DigitalInput(7);
		cubeSwitch = new DigitalInput(8);

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
		
		// Set up port for Ultrasonic Distance Sensor
		ultrasonic_yellow = new AnalogInput(0);
		ultrasonic_black = new AnalogInput(1);
		
		// Set up Encoder ports
		leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(2,3);
		elevatorEncoder = new Encoder(8,9);
		
		//Gyroscope Setup
		//gyroscope = new ADXRS450_Gyro();
		gyroscope = new AHRS(SPI.Port.kMXP);
		//gyroscope.calibrate();
	
		// Initialize ADB Communication 
		
		if (initializeADB){
			System.out.println("Initializing adb");
			RIOdroid.init();
			RIOadb.init();
			System.out.println("adb Initialized");
			
			System.out.println("Begin ADB Tests");
			System.out.println("Start ADB" + RIOdroid.executeCommand("adb start-server"));
			//System.out.println("LOGCAT: " + RIOdroid.executeCommand("adb logcat -t 200 ActivityManager:I native:D *:S"));
			System.out.println("logcat done");
		}
		else{
			System.out.println("ADB INIT NOT RAN");
		}
	}
	
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		autoStep = Step.STRAIGHT;
		resetEncoders();
		gyroscope.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		updateSmartDashboard();
		if (m_autoSelected == SquareAuto) {
			System.out.println("Square Auto Is Operating");
			switch(autoStep){
				case STRAIGHT:
					if (getDistance() < 24){
						driveStraight(0, 0.3);
						System.out.println("Going Straight");
					}
					else{
						stop();
						resetEncoders();
						autoStep = Step.TURN;
						System.out.println("Encoders Reset!");
					}
					
					break;
				case TURN:
					if(turnRight(90)){
						stop();
						resetEncoders();
						gyroscope.reset();
						autoStep = Step.STRAIGHT;
						System.out.println("Turned Right");
					}
					break;
			}
		}
		
		return;
	}

	/**
	 * This function is called periodically during operator control.
	 */
	// TODO Jump to periodic
	@Override
	public void teleopPeriodic() {
		
		chassis.arcadeDrive(driver.getX(), -driver.getY());
		//tipPrevention();
		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		
		aButtonOp = operator.getRawButton(1);
		bButtonOp = operator.getRawButton(2);
		xButtonOp = operator.getRawButton(3);
		yButtonOp = operator.getRawButton(4);

		// debug for tipPrevention
		//System.out.println(gyroscope.getPitch());
		
		if (xButton) {
			//System.out.print(androidData());
			elevatorEncoder.reset();
			resetEncoders();
			gyroscope.reset();
		} 
		
		if (aButton){
			turnToBox();
		}
		
		if (aButtonOp) {
			gripperGroup.set(1);
		} 
		else if (bButtonOp) {
			gripperGroup.set(-1);
		}
		else {
			gripperGroup.set(0);
		}
		
		
		if (limitSwitchUpElevator.get() & limitSwitchDownElevator.get()) {
			elevator.set(operator.getRawAxis(1));
		}
		if (!limitSwitchUpElevator.get() & (operator.getRawAxis(1) > 0)) { 
 			elevator.set(0);
 			System.out.println("UP ELEVATOR limit Switct being pressed while DOWN Joystick is pressed");
 		}
		if (!limitSwitchDownElevator.get() & (operator.getRawAxis(1) < 0)) { 
 			elevator.set(0);
 			System.out.println("DOWN ELEVATOR limit Switct being pressed while UP Joystick is pressed");
 		}
		//else {
			//elevatorGroup.set(0);
		//}
 			/*
		if (limitSwitchUpClimber.get() & limitSwitchDownClimber.get()) {
			climberGroup.set(operator.getRawAxis(5));
		}
		
		if (!limitSwitchUpClimber.get() & (operator.getRawAxis(5) > 0)) { 
 			climberGroup.set(0);
 			System.out.println("UP CLIMBER limit Switct being pressed while DOWN Joystick is pressed");	
 		}
		if (!limitSwitchDownClimber.get() & (operator.getRawAxis(5) < 0)) {
			climberGroup.set(0);
 			System.out.println("DOWN CLIMBER limit Switct being pressed while UP Joystick is pressed");	
 		}
		//else {
			//climberGroup.set(0); 
		//}*/

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
	
	// Calculates and returns the distance from the Yellow Ultrasonic Distance Sensor
	public double getUltrasonicYellowDistance(){
		// Calculates distance in centimeters from ultrasonic distance sensor
		return (double)(((ultrasonic_yellow.getAverageVoltage()*1000)/238.095)+9.0); //accuracy of 2 millimeters ;)
	}

	// Calculates and returns the distance from the Black Ultrasonic Distance Sensor
	public double getUltrasonicBlackDistance(){
		// Calculates distance in centimeters from ultrasonic distance sensor
		return (double)(((ultrasonic_black.getAverageVoltage()*1000)/9.4));
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
			double degreeOffset = (170 - Center_X)*DEGREES_PER_PIXEL_LEFT;
			double boxAngle = gyroscope.getAngle()%360 + degreeOffset;

			System.out.println("Should be turning left, Offset: " + degreeOffset);
			while (gyroscope.getAngle()%360 > boxAngle){
				turnLeft(boxAngle);
			}
		}
		if (Center_X < 170 && Center_X != 0){
			double degreeOffset = (170 - Center_X)*DEGREES_PER_PIXEL_RIGHT;
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
		leftBack.set(0.35);
		leftFront.set(0.35);
		rightBack.set(0.35);
		rightFront.set(0.35);

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
		leftBack.set(-0.35);
		leftFront.set(-0.35);
		rightBack.set(-0.35);
		rightFront.set(-0.35);

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
		
		SmartDashboard.putNumber("Ultrasonic Yellow Distance (cm):", getUltrasonicYellowDistance());
		SmartDashboard.putNumber("Ultrasonic Black Distance (cm):", getUltrasonicBlackDistance());
		
		SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
		
		SmartDashboard.putBoolean("Cube Held", !(cubeSwitch.get()));
		
		SmartDashboard.putNumber("Robot Speed", robotSpeed());
		
	}
}
