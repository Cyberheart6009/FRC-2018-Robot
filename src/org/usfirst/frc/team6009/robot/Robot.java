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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team6009.robot.Robot.Step;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.*;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import org.spectrum3847.RIOdroid.RIOadb;
import org.spectrum3847.RIOdroid.RIOdroid;


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
public class Robot extends IterativeRobot implements PIDOutput {	
	String gameData;
	// Auto Modes Setup
	// Create Position Chooser and its Choices
	SendableChooser<String> positionChooser;
	private static final String left = "left";
	private static final String center = "center";
	private static final String right = "right";
	private static final String square = "square";
	private static final String leftSwitch = "leftSwitch";
	private static final String leftSwitchSwitch = "leftSwitchSwitch";
	private static final String rightSwitchSwitch = "rightSwitchSwitch";
	// Creating Movement Chooser and its choices
	SendableChooser<String> movementChooser;
	private static final String switchSwitch = "switchSwitch";
	private static final String switchScale = "switchScale";
	private static final String scaleSwitch = "scaleSwitch";
	private static final String scaleScale = "scaleScale";
	private static final String straight = "straight";
	String positionSelected;
	String movementSelected;
	
	//auto cases
	// Turn2, Turn3, Turn4, Straight2, Straight3, Straight4
	public enum Step { 
		Straight1, Straight2, Straight3, Straight4, Straight5, Straight6,
		Turn1, Turn2, Turn3, Turn4, 
		Elevator1, Elevator2, Elevator3,
		Gripper1, Gripper2,
		Cube1, Cube2,
		CubeOut, CubeOut2,
		CubeIn,
		GripStraight1,
		Done 
		}
	public Step autoStep = Step.Straight1;
	public long timerStart;
	
	//Controller Inversion
	public enum isInverted {
		TRUE,
		FALSE
	}
	
	public isInverted controlInvert = isInverted.FALSE;

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
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, climberOne, climberTwo, elevatorOne, elevatorTwo, gripperOne, gripperTwo;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, elevatorGroup, climberGroup, gripperGroup;

	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	// Joystick Definitions
	Joystick driver;
  Joystick operator;
	
	//Limit Switch
	DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber;
	
	// Boolean for buttons
	boolean aButton, bButton, yButton, xButton, leftBumper, rightBumper, start, select, leftThumbPush, rightThumbPush, aButtonOp, bButtonOp, xButtonOp, yButtonOp;;
	
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	Solenoid ultra_solenoid;
	
	// Encoders
	Encoder leftEncoder, rightEncoder, elevatorEncoder;
	
	// Gyro
	//ADXRS450_Gyro gyroscope;
	AHRS gyroscope;
	
	//PID Variables
	PIDController rotationPID;
	
	// CONSTANT VARIABLES FOR PID
	//double KpTurn = 0.075;
	double Kp = 0.03;
	//double Ki = 0;
	//double Kd = 0.195;
	//double Kd = 0.0195;
	
	double KpTurn = 0.010;
	double Ki = 0.0;
	double Kd = 0.0195;
	
	//Square movement variables
	int squareStep;
	int side;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create Position Types
		positionChooser = new SendableChooser<String>();
		positionChooser.addObject("Left", left);
		positionChooser.addObject("Center", center);
		positionChooser.addObject("Right", right);
		positionChooser.addObject("Square", square);
		positionChooser.addObject("left Switch", leftSwitch);
		positionChooser.addObject("left Switch Switch", leftSwitchSwitch);
		positionChooser.addObject("right Switch Switch", rightSwitchSwitch);
		positionChooser.addObject("straight", straight);
		// Create Movement Types
		movementChooser = new SendableChooser<String>();
		movementChooser.addObject("Switch 2x", switchSwitch);
		movementChooser.addObject("Switch then Scale", switchScale);
		movementChooser.addObject("Scale then Switch", scaleSwitch);
		movementChooser.addObject("Scale 2x", scaleScale);
		//Display these choices in whatever interface we are using
		SmartDashboard.putData("Robot Position", positionChooser);
		SmartDashboard.putData("Movement Type", movementChooser);
		
		//Defines limit switch ports
		limitSwitchUpElevator = new DigitalInput(4);
		limitSwitchDownElevator =  new DigitalInput(5);
		limitSwitchUpClimber = new DigitalInput(6);
		limitSwitchDownClimber = new DigitalInput(7);
		
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
		elevatorOne.setInverted(true);
		gripperTwo.setInverted(true);
    
		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		elevatorGroup = new SpeedControllerGroup(elevatorOne, elevatorTwo);
		climberGroup = new SpeedControllerGroup(climberOne, climberTwo);
		gripperGroup = new SpeedControllerGroup(gripperOne, gripperTwo);
		
		// Defines Joystick ports
		driver = new Joystick(0);
		operator = new Joystick(1);
		
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
		elevatorEncoder = new Encoder(8,9);
		
		//Gyroscope Setup
		//gyroscope = new ADXRS450_Gyro();
		gyroscope = new AHRS(SPI.Port.kMXP);
		//gyroscope.calibrate();

		rotationPID = new PIDController(KpTurn, Ki, Kd, gyroscope, this);
    
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
		//Assign selected modes to a variable
		positionSelected = positionChooser.getSelected();
		movementSelected = movementChooser.getSelected();
		System.out.println("Position Selected: " + positionSelected);
		System.out.println("Movement Selected" + movementSelected);
		//Get Orientation of scale and store it in gameData
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		//Reset the gyro so the heading at the start of the match is 0
		resetEncoders();
		autoStep = Step.Straight1;
		gyroscope.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	/*public void autonomousPeriodic() {
		updateSmartDashboard();
		if (positionSelected == square) {
			System.out.println("Square Auto Is Operating");
			switch(autoStep){
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
						gyroscope.reset();
					}
					break;
				case Turn:
					if(turnRight(90)){
						stop();
						resetEncoders();
						gyroscope.reset();
						autoStep = Step.Straight;
						System.out.println("Turned Right");
					}
					/*if (gyroscope.getAngle() <= 89) {
						rotationPID.setSetpoint(90);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
					} else {
						resetEncoders();
						autoStep = Step.Straight;
						rotationPID.setEnabled(false);
						gyroscope.reset();
					}*/
					/*if (turnInPlace(90)) {
					  	resetEncoders();
						autoStep = Step.Straight;
						rotationPID.setEnabled(false);
						gyroscope.reset();
					}
					break;
			}
		}
		
		return;*/
	
	public void autonomousPeriodic() {
		double distance = getDistance();
		if (System.currentTimeMillis() - timerStart < 500) {
			return;
		}
		if (positionSelected.equalsIgnoreCase(square)) {
			System.out.println("Square Auto Is Operating");
			switch(autoStep){
				case Straight1:
					if (driveDistanceStraight(24,0.3)){
						autoStep = Step.Turn1;
						System.out.println("Moved Straight");
						timerStart = System.currentTimeMillis();
					}
					break;
				case Turn1:
					/*if(turnRight(90)){
						stop();
						resetEncoders();
						gyroscope.reset();
						autoStep = Step.Straight;
						System.out.println("Turned Right");
					}*/
					if (turnInPlace(90)) {
					  	resetEncoders();
						autoStep = Step.Straight1;
						rotationPID.setEnabled(false);
						gyroscope.reset();
						System.out.println("Turned Right");
						timerStart = System.currentTimeMillis();
					}
					break;
			}
		}
		/**km code begins**/
		double height = getElevatorHeight();
		if (positionSelected.equalsIgnoreCase(straight)){
			//driveStraight(0, 0.4);
			leftChassis.set(0.3);
			rightChassis.set(0.3);
			if (distance >= 70) {
				stop();
			}
		}
		if (positionSelected.equalsIgnoreCase(leftSwitchSwitch) || positionSelected.equalsIgnoreCase(rightSwitchSwitch)) {
			switch (autoStep) {
				case Straight1:
					resetEncoders();
					if (distance < 220) {
						driveStraight(0, 0.4);
					}
					else {
						stop();
						autoStep = Step.Turn1;
					}
					break;
				case Turn1: 
					if (positionSelected == leftSwitchSwitch){
						if (gyroscope.getAngle() >= (90 - 2) && gyroscope.getAngle() <= (90 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight2;
						}
						else {
							PIDTurn(90);
						}
					}
					else {
						if (gyroscope.getAngle() >= (-90 - 2) && gyroscope.getAngle() <= (-90 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight2;
						}
						else {
							PIDTurn(-90);
						}
					}
					break;
				case Straight2:
					if (positionSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						resetEncoders();
						if (distance < 45.5) {
							driveStraight(90, 0.4);
						}
						else {
							stop();
							autoStep = Step.Turn2;
						}
					}
					else {
						rotationPID.setEnabled(false);
						resetEncoders();
						if (distance < 45.5) {
							driveStraight(-90, 0.4);
						}
						else {
							stop();
							autoStep = Step.Turn2;
						}
					}
					break;
				case Turn2:
					if (positionSelected == leftSwitchSwitch) {
						if (gyroscope.getAngle() >= (180 - 2) && gyroscope.getAngle() <= (180 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight2;
						}
						else {
							PIDTurn(180);
						}
						autoStep = Step.Straight3;
					}
					else {
						if (gyroscope.getAngle() >= (-180 - 2) && gyroscope.getAngle() <= (-180 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight2;
						}
						else {
							PIDTurn(-180);
						}
						autoStep = Step.Straight3;
					}
					break;
				case Straight3:
					/* could use vision tracking of cube, at this point the cube should be about in front of and centered 
					in regards to our robot so using the tracking will make sure we are in line with the cube*/
					if (positionSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						resetEncoders();
						if (distance < 11){
							driveStraight(180, 0.4);
						}
						else {
							stop();
							autoStep = Step.CubeOut;
						}
					}
					else {
						rotationPID.setEnabled(false);
						resetEncoders();
						if (distance < 11){
						driveStraight(-180, 0.4);
						}
						else {
							stop();
							autoStep = Step.CubeOut;
						}
					}
					break;
				case CubeOut:
					if (height < 40) {
						elevatorGroup.set(0.3);
					}
					else {
						stopElevator();
					}
					/*gripper1.set(0.3);	
					gripper2.set(0.3);
					if (!limitSwitchGripper.get()) {
						gripper1.set(0);
						gripper2.set(0);
					}*/
					autoStep = Step.CubeIn;
					break;
				case CubeIn:
					rotationPID.setEnabled(false);
					resetEncoders();
					if (positionSelected == leftSwitchSwitch) {
						if (height < 0) {
							elevatorGroup.set(-0.1);
						}
						else {
							stopElevator();
						}
						if (limitSwitchDownElevator.get()) {
							stopElevator();
						}
						if (distance < 13) {
							driveStraight(180, 0.4);
						}
						else {
							stop();
						}
						/*gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}*/
						autoStep = Step.CubeOut2;
					}
					else {
						if (height < 0) {
							elevatorGroup.set(-0.1);
						}
						else {
							stopElevator();
						}
						if (limitSwitchDownElevator.get()) {
							stopElevator();
						}
						if (distance < 13) {
							driveStraight(-180, 0.4);
						}
						else {
							stop();
						}
						/*gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}*/
						autoStep = Step.CubeOut2;
					}
					break;
				case CubeOut2:
					if (height < 40) {
						elevatorGroup.set(0.3);
					}
					else {
						stopElevator();
					}
					/*gripper1.set(0.3);
					gripper2.set(0.3);
					if (!limitSwitchGripper.get()) {
						gripper1.set(0);
						gripper2.set(0);
					}*/
					autoStep = Step.Done;
					break;
				case Done:
					stop();
					stopElevator();
					stopGripper();
					break;
			}
		}
		/**km code ends*/
		//This code is activated if the robot is on the left or right sides. The modes will be split up later.
		if (positionSelected == "left" || positionSelected == "right") {
			//These are combined under 1 if section because they both start with switch
			if (movementSelected == "switchSwitch" || movementSelected == "switchScale") {
				if (gameData.charAt(0) == 'L') {
					// TODO Insert Code to move the Robot to the left side of the switch and drop a box into it
					if (movementSelected == "switchSwitch") {
						// TODO Insert Code to pick up box and plant it in switch
						
					}
					if (movementSelected == "switchScale" && gameData.charAt(1) == 'L') {
						// TODO Insert Code to pick up box and move to the left side of the scale
						
					}
					if (movementSelected == "switchScale" && gameData.charAt(1) == 'R') {
						// TODO Insert Code to pick up box and move to the right side of the scale
					}
				}
			}
			//These are combined under 1 if section because they both start with scale
			if (movementSelected == "ScaleScale" || movementSelected == "ScaleSwitch") {
				
			}
		}
		//This code is activated if the robot is in the center
		if (positionSelected == "center") {
			//These are combined under 1 if section because they both start with switch
			if (movementSelected == "switchSwitch" || movementSelected == "switchScale") {
				switch (autoStep) {
				case Straight1:
					resetEncoders();
					if (driveDistanceStraight(70,0.3)){
						autoStep = Step.Turn1;
						System.out.println("Moved Straight");
						timerStart = System.currentTimeMillis();
					}
					break;
				}
				if (gameData.charAt(0) == 'L') {
					// TODO Insert Code to pick up box and move to the left side of the scale
					switch (autoStep) {
					case Turn1:
						if (turnInPlace(90)) {
						  	resetEncoders();
							autoStep = Step.Straight2;
							System.out.println("Turned Right");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight2:
						resetEncoders();
						if (driveDistanceStraight(59,0.3)){
							autoStep = Step.Turn2;
							System.out.println("Moved Straight");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn2:
						if (turnInPlace(0)) {
						  	resetEncoders();
							autoStep = Step.Straight3;
							System.out.println("Turned Left");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight3:
						if (driveDistanceStraight(59,0.3)){
							autoStep = Step.Elevator1;
							System.out.println("Moved Straight");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Elevator1:
						if (elevatorAscend(20, 0.8)) {
							autoStep = Step.Gripper1;
							System.out.println("Elevator Ascended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Gripper1:
						if (System.currentTimeMillis() - timerStart >= 3000) {
							autoStep = Step.Gripper1;
							System.out.println("Gripper Outed");
							timerStart = System.currentTimeMillis();
						} else {
							gripperGroup.set(0.3);
						}
						break;
					case Elevator2:
						if (elevatorDescend(0, 0.8)) {
							autoStep = Step.Straight4;
							System.out.println("Elevator Descended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight4:
						if (driveDistanceStraight(-10, -0.3)) { 
							resetEncoders();
							autoStep = Step.Turn3;
							System.out.println("Moved 10 inches backwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn3:
						if (turnInPlace(90)) {
							resetEncoders();
							autoStep = Step.GripStraight1;
							System.out.println("Turned in place to 90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case GripStraight1:
						if (driveDistanceStraight(59,0.3)){ // FIXME Fix this distance value
							autoStep = Step.Done;
							System.out.println("Gripper Activated and moved straight");
							timerStart = System.currentTimeMillis();
						} else {
							gripperGroup.set(0.3);
						}
						break;
					case Done:
						System.out.println("Auto Has finished");
						break;
					}
					if (movementSelected == "switchSwitch") {
						switch (autoStep) {
						case Straight5:
							if (driveDistanceStraight(-10, -0.3)) { // FIXME Set this distance value to negative the one above
								resetEncoders();
								autoStep = Step.Turn4;
								System.out.println("Moved 10 inches backwards");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn4:
							if (turnInPlace(0)) {
								resetEncoders();
								autoStep = Step.Straight6;
								System.out.println("Turned in place to 90 degrees");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight6:
							if (driveDistanceStraight(10, 0.3)) { // FIXME Set this distance value to negative the one above
								resetEncoders();
								autoStep = Step.Elevator3;
								System.out.println("Moved 10 inches backwards");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Elevator3:
							if (elevatorAscend(20, 0.8)) {
								autoStep = Step.Gripper2;
								System.out.println("Elevator Ascended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Gripper2:
							if (System.currentTimeMillis() - timerStart >= 3000) {
								autoStep = Step.Gripper1;
								System.out.println("Gripper Outed");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(0.3);
							}
							break;
						}
					}
					
				}
				if (gameData.charAt(0) == 'R') {
					// TODO Insert Code to pick up box and move to the right side of the scale
				}
				
			}
			//These are combined under 1 if section because they both start with scale
			if (movementSelected == "ScaleScale" || movementSelected == "ScaleSwitch") {
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
		ultra_solenoid.set(true);
		//leftChassis.set(0.1);
		//rightChassis.set(0.1);
		
		if (controlInvert == isInverted.FALSE) {
			chassis.arcadeDrive(driver.getX(), -driver.getY());
		} 
		else if (controlInvert == isInverted.TRUE) {
			chassis.arcadeDrive(-driver.getX(), driver.getY());
		}
    
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
    aButtonOp = operator.getRawButton(1);
		bButtonOp = operator.getRawButton(2);
		xButtonOp = operator.getRawButton(3);
		yButtonOp = operator.getRawButton(4);
		
		SmartDashboard.putNumber("Ultrasonic Yellow Distance (cm):", getUltrasonicYellowDistance());
		SmartDashboard.putNumber("Ultrasonic Black Distance (cm):", getUltrasonicBlackDistance());
		
		/*if(rotationPID.isEnabled()){
			rightChassis.set(-(rotationPID.get()));
			leftChassis.set((rotationPID.get()));
		}*/
		double distance = getDistance();
		if (bButton){
			rotationPID.setEnabled(false);
		} 
		else if (aButton) {
		}
		if(xButton){
			gyroscope.reset();
			resetEncoders();
		}
		if(yButton){
			
			driveStraight(0, 0.3);
			if (distance >= 24) {
				stop();
				System.out.println("DriveStraight");
			}
		}
		
		if (start) {
			//Invert the controls
			if (controlInvert == isInverted.FALSE) {
				controlInvert = isInverted.TRUE;
			} else if (controlInvert == isInverted.TRUE) {
				controlInvert = isInverted.TRUE;
			}
		}
		/**Change PID Values**/
		/*if (leftBumper) {
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
		}*/
		
		//System.out.println(rotationPID.get());
		updateSmartDashboard();
	}

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
	    gripper.set(0);
		}
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
	
	public boolean turnInPlace(double setPoint) {
		/* XXX This might be fixed: Setpoint is currently required to be exact, implement a range*/
		if (gyroscope.getAngle() >= (setPoint - 2) && gyroscope.getAngle() <= (setPoint + 2)) {
			return true;
		} else {
			rotationPID.setSetpoint(setPoint);
			rotationPID.setEnabled(true);
			leftChassis.set(rotationPID.get());
			rightChassis.set(-rotationPID.get());
			rotationPID.setEnabled(true);
			return false;
		}
		
	}
	
	public boolean driveDistanceStraight(double distance, double speed) {
		System.out.println("driveDistanceStraight()");
		if (getDistance() < distance){
			driveStraight(0, speed);
			return false;
		}
		else {
			System.out.println("current distance end");
			stop();
			resetEncoders();
			return true;
		}
	}
	
	public boolean elevatorAscend(double height, double speed) {
		System.out.println("elevatorClimbHeight()");
		if (getElevatorHeight() < height){
			elevatorGroup.set(speed);
			return false;
		}
		else {
			System.out.println("Elevator Has Ascended");
			stop();
			resetEncoders();
			return true;
		}
	}
	
	public boolean elevatorDescend(double height, double speed) {
		System.out.println("elevatorClimbHeight()");
		if (getElevatorHeight() > height){
			elevatorGroup.set(-speed);
			return false;
		}
		else {
			System.out.println("Elevator Has Ascended");
			stop();
			resetEncoders();
			return true;
		}
	}
	
	public void squareDrive() {
		switch (squareStep){
		case 0:
			resetEncoders();
			rotationPID.setEnabled(false);
			driveDistanceStraight(30, 0.4);
			squareStep = 1;
			side += 1;
			//if (side == 4) {return;}
			break;
		case 1:
			turnInPlace(90);
			squareStep = 0;
			break;
		default:
			System.out.println("The squareDrive() function could not find a valid squareStep int");
		}
	}
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
	
	public double getElevatorHeight(){
		return ((double)(elevatorEncoder.get() / (ELEVATOR_ENCODER_COUNTS_PER_INCH * 2)));
	}
	
	public double getDistance(){
		//return ((double)(leftEncoder.get() + rightEncoder.get())) / (ENCODER_COUNTS_PER_INCH * 2);
		return (double)((leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
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
			leftSpeed += error * Kp;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * Kp;
		}
	
		// set the motors based on the inputed speed
		leftChassis.set(leftSpeed);
		rightChassis.set(rightSpeed);
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
	
	private void PIDTurn(double setpoint) {
		resetEncoders();
		rotationPID.setSetpoint(setpoint);
		rotationPID.setEnabled(true);
		leftChassis.set(rotationPID.get());
		rightChassis.set(-rotationPID.get());
	}
	
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
		elevatorOne.set(0);
		elevatorTwo.set(0);
	}
	
	private void stopElevator() {
		elevatorOne.set(0);
		elevatorTwo.set(0);
	}
	private void stopGripper() {
		gripperOne.set(0);
		gripperTwo.set(0);
	}
	
	@Override
	public void pidWrite(double output) {
		// TODOish Auto-generated method stub
    
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
		
		SmartDashboard.putNumber("Robot Speed", robotSpeed());
		
	}
}


//Legacy Autonomous & km Square Auto
/**public void autonomousPeriodic() {
	double distance = getDistance();
	if (positionSelected.equalsIgnoreCase(square)) {
		switch (autoStep) {
		case Straight:
			
			driveStraight(0, 0.3);
			if (distance > 10) {
				stop();
			}
			timerStart = System.currentTimeMillis();
			autoStep = Step.Turn;
			break;
		case Turn:
			rotationPID.setEnabled(true);
			rotationPID.setSetpoint(90);
			leftChassis.set(rotationPID.get());
			rightChassis.set(-rotationPID.get());
			rotationPID.setEnabled(false);
			
			timerStart = System.currentTimeMillis();
			autoStep = Step.Straight2;
			break;
		case Straight2:
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
			break;
		case Done:
			leftChassis.set(0);
			rightChassis.set(0);
			break;
		}
	}
	if(gameData.charAt(0) == 'L'){
		if (positionSelected.equalsIgnoreCase(leftSwitch)) {
			// for LL or LR
			switch (autoStep) {
			case Straight:
				driveStraight(0, 0.4);
				
				if (distance > 10){
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
}*/