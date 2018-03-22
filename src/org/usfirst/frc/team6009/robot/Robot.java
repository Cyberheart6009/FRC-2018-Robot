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
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.can.*;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
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
	private static final String straight = "straight";
	private static final String centerSwitchSwitch = "centerSwitchSwitch";
	private static final String centerSwitchScale = "centerSwitchScale";
	private static final String centerScaleScale = "centerScaleScale";
	private static final String centerScaleSwitch = "centerScaleSwitch";
	// Creating Movement Chooser and its choices
	//SendableChooser<String> movementChooser;
	//private static final String switchSwitch = "switchSwitch";
	//private static final String switchScale = "switchScale";
	//private static final String scaleSwitch = "scaleSwitch";
	//private static final String scaleScale = "scaleScale";
	String positionSelected;
	//String movementSelected;
	
	//auto cases
	// Turn2, Turn3, Turn4, Straight2, Straight3, Straight4
	/**"Name"1, "Name"2, etc... are first created cases, "Name"1m1, "Name"1m2, etc.. prevent the convention from breaking*/ 
	public enum Step { 
		Straight1, Straight2, Straight3, Straight3m1, Straight3m2, Straight4, Straight4m1, Straight4m2, Straight5, Straight5m2, Straight6, Straight7, Straight8, Straight9, Straight10, Straight11,
		Turn1, Turn2, Turn3, Turn3m1, Turn3m2, Turn3m3, Turn4, Turn5, Turn6, Turn7, Turn8, Turn9,
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
	public long timerInvert;
	
	//Controller Inversion
	public enum isInverted {
		TRUE,
		FALSE
	}
	
	public isInverted controlInvert = isInverted.FALSE;

	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, climberOne, climberTwo, elevatorOne, elevatorTwo, gripperOne, gripperTwo;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, elevatorGroup, climberGroup, gripperGroup;
	
	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	// Joystick Definitions
	Joystick driver;
	
	//Limit Switch
	//DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber, limitSwitchGripper;
	
	// Boolean for buttons
	boolean aButton, bButton, yButton, xButton, leftBumper, rightBumper, start, select, leftThumbPush, rightThumbPush;
	
	// Analog Sensors
	AnalogInput ultrasonic_yellow, ultrasonic_black;
	//Solenoid ultra_solenoid;
	
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
	
	double KpTurn = 0.017;
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
		//positionChooser.addDefault("Center", center);
		//positionChooser.addObject("Right", right);
		//positionChooser.addObject("Square", square);
		positionChooser.addObject("left Switch", leftSwitch);
		positionChooser.addObject("left Switch Switch", leftSwitchSwitch);
		positionChooser.addObject("right Switch Switch", rightSwitchSwitch);
		positionChooser.addObject("center Switch Switch", centerSwitchSwitch);
		positionChooser.addObject("center Switch Scale", centerSwitchScale);
		positionChooser.addObject("center Scale Switch", centerScaleSwitch);
		positionChooser.addObject("center Scale Scale", centerScaleScale);
		positionChooser.addObject("straight", straight);
		//positionChooser.addObject("Left", left);
		// Create Movement Types
		/*movementChooser = new SendableChooser<String>();
		movementChooser.addObject("Switch then Scale", switchScale);
		movementChooser.addObject("Scale then Switch", scaleSwitch);
		movementChooser.addObject("Scale 2x", scaleScale);
		movementChooser.addDefault("Switch 2x", switchSwitch);*/
		//Display these choices in whatever interface we are using
		SmartDashboard.putData("Robot Position", positionChooser);
		//SmartDashboard.putData("Movement Type", movementChooser);
		
		//Defines limit switch ports
		/*limitSwitchUpElevator = new DigitalInput(4);
		limitSwitchDownElevator =  new DigitalInput(5);
		limitSwitchUpClimber = new DigitalInput(6);
		limitSwitchDownClimber = new DigitalInput(7);
		limitSwitchGripper = new DigitalInput(8);*/
		
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
		
		//Creating the Joystick object
		driver = new Joystick(0);
		
		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		elevatorGroup = new SpeedControllerGroup(elevatorOne, elevatorTwo);
		climberGroup = new SpeedControllerGroup(climberOne, climberTwo);
		gripperGroup = new SpeedControllerGroup(gripperOne, gripperTwo);
		
		// Inverts the right side of the drive train to account for the motors being physically flipped
		rightChassis.setInverted(true);
		
		// Defines our DifferentalDrive object with both sides of our drivetrain
		chassis = new DifferentialDrive(leftChassis, rightChassis);
		
		// Set up port for Ultrasonic Distance Sensor
		ultrasonic_yellow = new AnalogInput(0);
		ultrasonic_black = new AnalogInput(1);
		//ultra_solenoid = new Solenoid(0);
		
		
		// Set up Encoder ports
		leftEncoder = new Encoder(0,1);
		rightEncoder = new Encoder(2,3);
		elevatorEncoder = new Encoder(8,9);
		//leftElevator = new Encoder(4, 5);
		//rightElevator = new Encoder(6, 7);
		
		//Gyroscope Setup
		//gyroscope = new ADXRS450_Gyro();
		gyroscope = new AHRS(SPI.Port.kMXP);
		//gyroscope.calibrate();
		
		rotationPID = new PIDController(KpTurn, Ki, Kd, gyroscope, this);
		
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
		//movementSelected = movementChooser.getSelected();
		System.out.println("Position Selected: " + positionSelected);
		//System.out.println("Movement Selected" + movementSelected);
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
		updateSmartDashboard();
		double distance = getDistance();
		System.out.println(distance);
		/*if (System.currentTimeMillis() - timerStart < 500) {
			return;
		}*/
		if (positionSelected.equalsIgnoreCase(square)) {
			System.out.println("Square Auto Is Operating");
			switch(autoStep){
				case Straight1:
					if (driveDistanceStraight(0,24,0.3)){
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
							resetEncoders();
						}
						else {
							PIDTurn(90);
						}
					}
					else {
						if (gyroscope.getAngle() >= (-90 - 2) && gyroscope.getAngle() <= (-90 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight2;
							resetEncoders();
						}
						else {
							PIDTurn(-90);
						}
					}
					break;
				case Straight2:
					if (positionSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						if (distance < 50) {
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
							autoStep = Step.Straight3;
							resetEncoders();
						}
						else {
							PIDTurn(180);
						}
					}
					else {
						if (gyroscope.getAngle() >= (-180 - 2) && gyroscope.getAngle() <= (-180 + 2) ){
							rotationPID.setEnabled(false);
							autoStep = Step.Straight3;
							resetEncoders();
						}
						else {
							PIDTurn(-180);
						}
					}
					break;
				case Straight3:
					/* could use vision tracking of cube, at this point the cube should be about in front of and centered 
					in regards to our robot so using the tracking will make sure we are in line with the cube*/
					if (positionSelected == leftSwitchSwitch) {
						rotationPID.setEnabled(false);
						if (distance < 11){
							driveStraight(180, 0.4);
						}
						else {
							stop();
							autoStep = Step.CubeIn;
							resetEncoders();
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
							autoStep = Step.CubeIn;
							resetEncoders();
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
					if (positionSelected == leftSwitchSwitch) {
						/*if (height < 0) {
							elevatorGroup.set(-0.1);
						}
						else {
							stopElevator();
						}
						if (limitSwitchDownElevator.get()true) {
							stopElevator();
						}*/
						if (distance < 13) {
							driveStraight(180, 0.4);
						}
						else {
							stop();
							autoStep = Step.Done;
						}
						/*gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}*/
						
					}
					else {
						/*if (height < 0) {
							elevatorGroup.set(-0.1);
						}
						else {
							stopElevator();
						}
						if (limitSwitchDownElevator.get()true) {
							stopElevator();
						}*/
						if (distance < 13) {
							driveStraight(-180, 0.4);
						}
						else {
							stop();
							autoStep = Step.Done;
						}
						/*gripper1.set(-0.3);
						gripper2.set(-0.3);
						if (limitSwitchGripper.get()) {
							gripper1.set(0);
							gripper2.set(0);
						}*/
						
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
		/*if (positionSelected == "left" || positionSelected == "right") {
			//These are combined under 1 if section because they both start with switch
			if (movementSelected == "switchSwitch" || movementSelected == "switchScale") {
				switch (autoStep) {
				case Straight1:
					if (driveDistanceStraight(0, 220, 0.5)){
						autoStep = Step.Turn1;
						System.out.println("Moved Straight");
						timerStart = System.currentTimeMillis();
					}
					break;
				case Turn1:
					if (positionSelected == "left") {
						if (turnInPlace(90)) {
						  	resetEncoders();
							autoStep = Step.Straight2;
							System.out.println("Turned to 90");
							timerStart = System.currentTimeMillis();
						}
					}
					if (positionSelected == "right") {
						if (turnInPlace(-90)) {
						  	resetEncoders();
							autoStep = Step.Straight2;
							System.out.println("Turned to -90");
							timerStart = System.currentTimeMillis();
						}
					}
					break;
				}
					if (gameData.charAt(0) == 'L') {
						switch (autoStep) {
						case Straight2:
							if (positionSelected == "left") {
								if (driveDistanceStraight(90, 50, 0.5)){
									autoStep = Step.Turn2;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-90, 246, 0.5)){
									autoStep = Step.Turn2;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							break;
						case Turn2:
							if (positionSelected == "left") {
								if (turnInPlace(180)) {
								  	resetEncoders();
									autoStep = Step.Straight3;
									System.out.println("Turned to 180");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (turnInPlace(-180)) {
								  	resetEncoders();
									autoStep = Step.Straight3;
									System.out.println("Turned to -180");
									timerStart = System.currentTimeMillis();
								}
							}
							break;
						case Straight3:
							if (positionSelected == "left") {
								if (driveDistanceStraight(180, 11, 0.5)){
									autoStep = Step.Elevator1;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-180, 11, 0.5)){
									autoStep = Step.Elevator1;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
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
								autoStep = Step.Elevator2;
								System.out.println("Gripper Outed");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(1);
							}
							break;
						case Elevator2:
							if (elevatorDescend(0, 0.8)) {
								resetEncoders();
								autoStep = Step.GripStraight1;
								System.out.println("Elevator Descended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case GripStraight1:
							if (positionSelected == "left") {
								if (driveDistanceStraight(180, 13, 0.3)){
									resetEncoders();
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Elevator3;
									}
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Straight4;
									}
									System.out.println("Gripper Activated and moved straight");
									timerStart = System.currentTimeMillis();
								} else {
									gripperGroup.set(-1);
								}
								break;
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-180, 13, 0.3)){
									resetEncoders();
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Elevator3;
									}
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Straight4;
									}
									System.out.println("Gripper Activated and moved straight");
									timerStart = System.currentTimeMillis();
								} else {
									gripperGroup.set(-1);
								}
								break;
							}
						}
							if (movementSelected == "switchSwitch") {
								switch (autoStep) {
								case Elevator3:
									if (elevatorAscend(20, 0.8)) {
										autoStep = Step.Gripper2;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto left or right, switchSwitch, gameData: L__ has been completed");
									break;
								}
								
							}
							if (movementSelected == "switchScale") {
								if (gameData.charAt(1) == 'L') {
									switch (autoStep) {
									case Straight4:
										if (positionSelected == "left") {
											if (driveDistanceStraight(180, -24, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-180, -24, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn3:
										if (positionSelected == "left") {
											if (turnInPlace(270)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to 270");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(-90)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to -270");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight5:
										if (positionSelected == "left") {
											if (driveDistanceStraight(270, 60, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-90, 60, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn4:
										if (positionSelected == "left") {
											if (turnInPlace(360)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 360");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(0)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to -360");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight6:
										if (positionSelected == "left") {
											if (driveDistanceStraight(360, 95, 0.5)){
												autoStep = Step.Turn5;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(0, 95, 0.5)){
												autoStep = Step.Turn5;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn5:
										if (positionSelected == "left") {
											if (turnInPlace(450)) {
											  	resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned to 450");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(90)) {
											  	resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned to -450");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight7:
										if (positionSelected == "left") {
											if (driveDistanceStraight(450, 32, 0.5)){ //XXX: The distance is probably wrong
												autoStep = Step.Elevator3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(90, 32, 0.5)){ //XXX: The distance is probably wrong
												autoStep = Step.Elevator3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
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
											autoStep = Step.Done;
											System.out.println("Gripper Outed");
											timerStart = System.currentTimeMillis();
										} else {
											gripperGroup.set(1);
										}
										break;
									case Done:
										System.out.println("Auto left or right, switchSwitch, gameData: LL_ has been completed");
										break;
									}
								}
								if (gameData.charAt(1) == 'R') {
									switch (autoStep) {
									case Straight4:
										if (positionSelected == "left") {
											if (driveDistanceStraight(180, -23, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-180, -23, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn3:
										if (positionSelected == "left") {
											if (turnInPlace(90)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to 90");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(90)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to -90");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight5:
										if (positionSelected == "left") {
											if (driveDistanceStraight(90, 214, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(90, 214, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn4:
										if (positionSelected == "left") {
											if (turnInPlace(0)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 360");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(0)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to -360");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight6:
										if (driveDistanceStraight(0, 95, 0.5)){
											autoStep = Step.Turn5;
											System.out.println("Moved Straight");
											timerStart = System.currentTimeMillis();
										}
										break;
									case Turn5:
											if (turnInPlace(-90)) {
											  	resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned to 450");
												timerStart = System.currentTimeMillis();
											}
										break;
									case Straight7:
										if (driveDistanceStraight(-90, 32, 0.5)){ //The distance is probably wrong
											autoStep = Step.Elevator3;
											System.out.println("Moved Straight");
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
											autoStep = Step.Done;
											System.out.println("Gripper Outed");
											timerStart = System.currentTimeMillis();
										} else {
											gripperGroup.set(1);
										}
										break;
									case Done:
										System.out.println("Auto left or right, switchSwitch, gameData: LR_ has been completed");
										break;
									}
								}
							}
					}
					//Our switch is on the right
					if (gameData.charAt(0) == 'R') {
						switch (autoStep) {
						case Straight2:
							if (positionSelected == "left") {
								if (driveDistanceStraight(90, 246, 0.5)){
									autoStep = Step.Turn2;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-90, 50, 0.5)){
									autoStep = Step.Turn2;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							break;
						case Turn2:
							if (positionSelected == "left") {
								if (turnInPlace(180)) {
								  	resetEncoders();
									autoStep = Step.Straight3;
									System.out.println("Turned to 180");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (turnInPlace(-180)) {
								  	resetEncoders();
									autoStep = Step.Straight3;
									System.out.println("Turned to -180");
									timerStart = System.currentTimeMillis();
								}
							}
							break;
						case Straight3:
							if (positionSelected == "left") {
								if (driveDistanceStraight(180, 11, 0.5)){
									autoStep = Step.Elevator1;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-180, 11, 0.5)){
									autoStep = Step.Elevator1;
									System.out.println("Moved Straight");
									timerStart = System.currentTimeMillis();
								}
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
								autoStep = Step.Elevator2;
								System.out.println("Gripper Outed");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(1);
							}
							break;
						case Elevator2:
							if (elevatorDescend(0, 0.8)) {
								resetEncoders();
								autoStep = Step.GripStraight1;
								System.out.println("Elevator Descended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case GripStraight1:
							if (positionSelected == "left") {
								if (driveDistanceStraight(180, 13, 0.3)){
									resetEncoders();
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Elevator3;
									}
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Straight4;
									}
									System.out.println("Gripper Activated and moved straight");
									timerStart = System.currentTimeMillis();
								} else {
									gripperGroup.set(-1);
								}
								break;
							}
							if (positionSelected == "right") {
								if (driveDistanceStraight(-180, 13, 0.3)){
									resetEncoders();
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Elevator3;
									}
									if (movementSelected == "switchSwitch") {
										autoStep = Step.Straight4;
									}
									System.out.println("Gripper Activated and moved straight");
									timerStart = System.currentTimeMillis();
								} else {
									gripperGroup.set(-1);
								}
								break;
							}
						}
							if (movementSelected == "switchSwitch") {
								switch (autoStep) {
								case Elevator3:
									if (elevatorAscend(20, 0.8)) {
										autoStep = Step.Gripper2;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto left or right, switchSwitch, gameData: L__ has been completed");
									break;
								}
								
							}
							if (movementSelected == "switchScale") {
								if (gameData.charAt(1) == 'R') {
									switch (autoStep) {
									case Straight4:
										if (positionSelected == "left") {
											if (driveDistanceStraight(180, -24, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-180, -24, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn3:
										if (positionSelected == "left") {
											if (turnInPlace(90)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to 90");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(-270)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to -270");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight5:
										if (positionSelected == "left") {
											if (driveDistanceStraight(90, 60, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-270, 60, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn4:
										if (positionSelected == "left") {
											if (turnInPlace(0)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 0");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(-360)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to -360");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight6:
										if (positionSelected == "left") {
											if (driveDistanceStraight(0, 95, 0.5)){
												autoStep = Step.Turn5;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-360, 95, 0.5)){
												autoStep = Step.Turn5;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn5:
										if (positionSelected == "left") {
											if (turnInPlace(-90)) {
											  	resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned to -90");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(-450)) {
											  	resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned to -450");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight7:
										if (positionSelected == "left") {
											if (driveDistanceStraight(-90, 32, 0.5)){ //XXX: The distance is probably wrong
												autoStep = Step.Elevator3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-450, 32, 0.5)){ //XXX: The distance is probably wrong
												autoStep = Step.Elevator3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
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
											autoStep = Step.Done;
											System.out.println("Gripper Outed");
											timerStart = System.currentTimeMillis();
										} else {
											gripperGroup.set(1);
										}
										break;
									case Done:
										System.out.println("Auto left or right, switchSwitch, gameData: LL_ has been completed");
										break;
									}
								}
								if (gameData.charAt(1) == 'L') {
									switch (autoStep) {
									case Straight4:
										if (positionSelected == "left") {
											if (driveDistanceStraight(180, -23, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-180, -23, -0.5)){
												autoStep = Step.Turn3;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn3:
										if (positionSelected == "left") {
											if (turnInPlace(270)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to 270");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(-90)) {
											  	resetEncoders();
												autoStep = Step.Straight5;
												System.out.println("Turned to -90");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight5:
										if (positionSelected == "left") {
											if (driveDistanceStraight(270, 214, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(-90, 214, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn4:
										if (positionSelected == "left") {
											if (turnInPlace(360)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 360");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(0)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 0");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight6:
										if (positionSelected == "left") {
											if (driveDistanceStraight(360, 95, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(0, 95, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Turn5:
										if (positionSelected == "left") {
											if (turnInPlace(450)) {
												resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 360");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (turnInPlace(90)) {
											  	resetEncoders();
												autoStep = Step.Straight6;
												System.out.println("Turned to 0");
												timerStart = System.currentTimeMillis();
											}
										}
										break;
									case Straight7:
										if (positionSelected == "left") {
											if (driveDistanceStraight(450, 32, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
										}
										if (positionSelected == "right") {
											if (driveDistanceStraight(90, 32, 0.5)){
												autoStep = Step.Turn4;
												System.out.println("Moved Straight");
												timerStart = System.currentTimeMillis();
											}
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
											autoStep = Step.Done;
											System.out.println("Gripper Outed");
											timerStart = System.currentTimeMillis();
										} else {
											gripperGroup.set(1);
										}
										break;
									case Done:
										System.out.println("Auto left or right, switchSwitch, gameData: LR_ has been completed");
										break;
									}
								}
							}
					}
			}
			//These are combined under 1 if section because they both start with scale
			if (movementSelected == "ScaleScale" || movementSelected == "ScaleSwitch") {
				//TODO The scale start stuff
			}
		}*/
			//These are combined under 1 if section because they both start with switch
			if (positionSelected == centerSwitchSwitch || positionSelected == centerSwitchScale) {
				switch (autoStep) {
				case Straight1:
					if (driveDistanceStraight(0, 30, 0.5)){
						autoStep = Step.Turn1;
						System.out.println("Moved Straight");
						timerStart = System.currentTimeMillis();
					}
					break;
				}
					//If our switch is on the left side
					if (gameData.charAt(0) == 'L') {
						switch (autoStep) {
						case Turn1:
							if (turnInPlace(-90)) {
							  	resetEncoders();
								autoStep = Step.Straight2;
								System.out.println("Turned Right");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight2:
							if (driveDistanceStraight(-90,40,0.5)){
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
						case Elevator1:
							if (elevatorAscend(50, 0.8)) {
								autoStep = Step.Straight3;
								System.out.println("Elevator Ascended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight3:
							if (driveDistanceStraight(0, 53, 0.5)){
								autoStep = Step.Straight3m1;
								resetEncoders();
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Gripper1:
							if (System.currentTimeMillis() - timerStart >= 3000) {
								resetEncoders();
								autoStep = Step.Straight3m1;
								System.out.println("Gripper Outed");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(1);
							}
							break;
						case Straight3m1:
							if (driveDistanceStraight(0, -53, -0.5)){
								autoStep = Step.Turn3;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Elevator2:
							if (elevatorDescend(0, 0.8)) {
								autoStep = Step.Turn3;
								System.out.println("Elevator Descended");
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
						case Straight3m2:
							if (driveDistanceStraight(90, 30, 0.5)){
								autoStep = Step.Turn3m1;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn3m1:
							if (turnInPlace(0)) {
								resetEncoders();
								autoStep = Step.GripStraight1;
								System.out.println("Turned in place to 90 degrees");
								timerStart = System.currentTimeMillis();
							}
							break;
						case GripStraight1:
							if (driveDistanceStraight(0, 20, 0.3)){
								resetEncoders();
								autoStep = Step.Straight5;
								System.out.println("Gripper Activated and moved straight");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(-1);
							}
							break;
						case Straight4:
							if (driveDistanceStraight(0, -10, -0.5)){
								autoStep = Step.Turn3m2;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn3m2:
							if (turnInPlace(-90)) {
								resetEncoders();
								autoStep = Step.Straight4m1;
								System.out.println("Turned in place to 90 degrees");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight4m1:
							if (driveDistanceStraight(-90, 30, 0.5)){
								autoStep = Step.Turn3m3;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn3m3:
							if (turnInPlace(0)) {
								resetEncoders();
								autoStep = Step.Straight4m2;
								System.out.println("Turned in place to 90 degrees");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight4m2:
							if (driveDistanceStraight(0, 43, 0.5)){
								autoStep = Step.Turn3m3;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						}
							//Code if the robot will drop another box on the switch.
							if (positionSelected == centerSwitchSwitch) {
								switch (autoStep) {
								case Straight5:
									if (driveDistanceStraight(90, -55.5, -0.5)) {
										resetEncoders();
										autoStep = Step.Turn4;
										System.out.println("Moved 55.5 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn4:
									if (turnInPlace(0)) {
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Turned in place to 90 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight5m2;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight5m2:
									if (driveDistanceStraight(10,15,0.5)) {
										resetEncoders();
										autoStep = Step.Turn4;
										System.out.println("Moved 55.5 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, SwitchSwitch, gameData: L__ has been completed");
									break;
								}
							}
							//Code for the robot to drop a box on the scale
							if (positionSelected == centerSwitchScale) {
								if (autoStep == Step.Straight5) {
									if (driveDistanceStraight(90, -120, -0.3)) {
										resetEncoders();
										autoStep = Step.Turn4;
										System.out.println("Moved 10 inches backwards");
										timerStart = System.currentTimeMillis();
									}
								}
									//Our scale is on the Left
									if (gameData.charAt(1) == 'L') {
										switch (autoStep) {
										case Turn4:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight6:
											if (driveDistanceStraight(0, 223, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn5;
												System.out.println("Moved 10 inches backwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn5:
											if (turnInPlace(90)) {
												resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight7:
											if (driveDistanceStraight(90, 26, 0.5)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Moved 26 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Elevator3:
											if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
												autoStep = Step.Gripper2;
												System.out.println("Elevator Ascended");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Gripper2:
											if (System.currentTimeMillis() - timerStart >= 3000) {
												autoStep = Step.Done;
												System.out.println("Gripper Outed");
												timerStart = System.currentTimeMillis();
											} else {
												gripperGroup.set(1);
											}
											break;
										case Done:
											System.out.println("Auto center, SwitchScale, gameData: LL_ has been completed");
											break;
										}
									}
									//Our scale is on the right
									if (gameData.charAt(1) == 'R') {
										switch (autoStep) {
										case Turn4:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight6:
											if (driveDistanceStraight(0, 86, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn5;
												System.out.println("Moved 86 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn5:
											if (turnInPlace(90)) {
												resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight7:
											if (driveDistanceStraight(90, 200, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn6;
												System.out.println("Moved 200 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn6:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Straight8;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight8:
											if (driveDistanceStraight(0, 80, 0.5)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Moved 10 inches backwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Elevator3:
											if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
												autoStep = Step.Gripper2;
												System.out.println("Elevator Ascended");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Gripper2:
											if (System.currentTimeMillis() - timerStart >= 3000) {
												autoStep = Step.Done;
												System.out.println("Gripper Outed");
												timerStart = System.currentTimeMillis();
											} else {
												gripperGroup.set(1);
											}
											break;
										case Done:
											System.out.println("Auto center, SwitchScale, gameData: LR_ has been completed");
											break;
										}
									}
							}
					}
					if (gameData.charAt(0) == 'R') {
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
							if (driveDistanceStraight(90,61,0.5)){
								autoStep = Step.Turn2;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn2:
							if (turnInPlace(0)) {
							  	resetEncoders();
								autoStep = Step.Elevator1;
								System.out.println("Turned Left");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Elevator1:
							if (elevatorAscend(50, 0.8)) {
								autoStep = Step.Straight3;
								System.out.println("Elevator Ascended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Straight3:
							if (driveDistanceStraight(0, 63, 0.5)){
								autoStep = Step.Gripper1;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Gripper1:
							if (System.currentTimeMillis() - timerStart >= 3000) {
								autoStep = Step.Straight3m1;
								System.out.println("Gripper Outed");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(1);
							}
							break;
						case Straight3m1:
							if (driveDistanceStraight(0, -15, -0.5)){
								autoStep = Step.Elevator2;
								System.out.println("Moved Straight");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Elevator2:
							if (elevatorDescend(0, 0.8)) {
								autoStep = Step.Turn3;
								System.out.println("Elevator Descended");
								timerStart = System.currentTimeMillis();
							}
							break;
						case Turn3:
							if (turnInPlace(-90)) {
								resetEncoders();
								autoStep = Step.GripStraight1;
								System.out.println("Turned in place to 90 degrees");
								timerStart = System.currentTimeMillis();
							}
							break;
						case GripStraight1:
							if (driveDistanceStraight(-90, 55.5, 0.3)){
								resetEncoders();
								autoStep = Step.Straight5;
								System.out.println("Gripper Activated and moved straight");
								timerStart = System.currentTimeMillis();
							} else {
								gripperGroup.set(-1);
							}
							break;
						case Done:
							System.out.println("Auto Has finished");
							break;
						}
							//Code if the robot will drop another box on the switch.
							if (positionSelected == centerSwitchSwitch) {
								switch (autoStep) {
								case Straight5:
									if (driveDistanceStraight(-90, -55.5, -0.5)) {
										resetEncoders();
										autoStep = Step.Turn4;
										System.out.println("Moved 10 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn4:
									if (turnInPlace(0)) {
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Turned in place to 90 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight5m2;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight5m2:
									if (driveDistanceStraight(0, 15, 0.5)) {
										resetEncoders();
										autoStep = Step.Gripper2;
										System.out.println("Moved 10 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, SwitchSwitch, gameData: R__ has been completed");
									break;
								}
							}
							//Code for the robot to drop a box on the scale
							if (positionSelected == centerSwitchScale) {
								if (autoStep == Step.Straight5) {
									if (driveDistanceStraight(-90, -120, -0.5)) {
										resetEncoders();
										autoStep = Step.Turn4;
										System.out.println("Moved 10 inches backwards");
										timerStart = System.currentTimeMillis();
									}
								}
									//Our scale is on the Right
									if (gameData.charAt(1) == 'R') {
										switch (autoStep) {
										case Turn4:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Turned in place to 90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight6:
											if (driveDistanceStraight(0, 223.5, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn5;
												System.out.println("Moved 223.5 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn5:
											if (turnInPlace(-90)) {
												resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned in place to -90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight7:
											if (driveDistanceStraight(-90, 26, 0.5)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Moved 26 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Elevator3:
											if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
												autoStep = Step.Gripper2;
												System.out.println("Elevator Ascended");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Gripper2:
											if (System.currentTimeMillis() - timerStart >= 3000) {
												autoStep = Step.Done;
												System.out.println("Gripper Outed");
												timerStart = System.currentTimeMillis();
											} else {
												gripperGroup.set(1);
											}
											break;
										case Done:
											System.out.println("Auto center, SwitchScale, gameData: RR_ has been completed");
											break;
										}
									}
									//Our scale is on the Left
									if (gameData.charAt(1) == 'L') {
										switch (autoStep) {
										case Turn4:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Turned in place to 0 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight6:
											if (driveDistanceStraight(0, 86, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn5;
												System.out.println("Moved 86 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn5:
											if (turnInPlace(-90)) {
												resetEncoders();
												autoStep = Step.Straight7;
												System.out.println("Turned in place to -90 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight7:
											if (driveDistanceStraight(-90, 200, 0.5)) {
												resetEncoders();
												autoStep = Step.Turn6;
												System.out.println("Moved 10 inches backwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Turn6:
											if (turnInPlace(0)) {
												resetEncoders();
												autoStep = Step.Straight8;
												System.out.println("Turned in place to 0 degrees");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Straight8:
											if (driveDistanceStraight(0, 80, 0.5)) {
												resetEncoders();
												autoStep = Step.Elevator3;
												System.out.println("Moved 80 inches forwards");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Elevator3:
											if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
												autoStep = Step.Gripper2;
												System.out.println("Elevator Ascended");
												timerStart = System.currentTimeMillis();
											}
											break;
										case Gripper2:
											if (System.currentTimeMillis() - timerStart >= 3000) {
												autoStep = Step.Done;
												System.out.println("Gripper Outed");
												timerStart = System.currentTimeMillis();
											} else {
												gripperGroup.set(1);
											}
											break;
										case Done:
											System.out.println("Auto center, SwitchScale, gameData: LR_ has been completed");
											break;
										}
									}
							}
					}
				
			}
			//These are combined under 1 if section because they both start with scale
			if (positionSelected == centerScaleScale || positionSelected == centerScaleSwitch) {
				switch (autoStep) {
				case Straight1:
					if (driveDistanceStraight(0, 43.5, 0.5)) {
						resetEncoders();
						autoStep = Step.Turn1;
						System.out.println("Moved 43.5 inches forwards");
						timerStart = System.currentTimeMillis();
					}
					break;
				}
				if (gameData.charAt(1) == 'L') {
					switch (autoStep) {
					case Turn1:
						if (turnInPlace(-90)) {
							resetEncoders();
							autoStep = Step.Straight2;
							System.out.println("Turned in place to -90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight2:
						if (driveDistanceStraight(-90, 125, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn2;
							System.out.println("Moved 125 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn2:
						if (turnInPlace(0)) {
							resetEncoders();
							autoStep = Step.Straight3;
							System.out.println("Turned in place to 0 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight3:
						if (driveDistanceStraight(0, 255.5, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn3;
							System.out.println("Moved 255.5 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn3:
						if (turnInPlace(90)) {
							resetEncoders();
							autoStep = Step.Straight3;
							System.out.println("Turned in place to 90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight4:
						if (driveDistanceStraight(90, 26, 0.5)) {
							resetEncoders();
							autoStep = Step.Elevator1;
							System.out.println("Moved 26 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Elevator1:
						if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
							autoStep = Step.Gripper1;
							System.out.println("Elevator Ascended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Gripper1:
						if (System.currentTimeMillis() - timerStart >= 3000) {
							autoStep = Step.Elevator2;
							System.out.println("Gripper Outed");
							timerStart = System.currentTimeMillis();
						} else {
							gripperGroup.set(1);
						}
						break;
					case Elevator2:
						if (elevatorDescend(0, 0.8)) {
							autoStep = Step.Straight5;
							System.out.println("Elevator Descended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight5:
						if (driveDistanceStraight(90, -32, -0.5)) {
							resetEncoders();
							autoStep = Step.Turn4;
							System.out.println("Moved 32 inches backwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn4:
						if (turnInPlace(180)) {
							resetEncoders();
							autoStep = Step.Straight6;
							System.out.println("Turned in place to 180 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight6:
						if (driveDistanceStraight(180, 95, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn5;
							System.out.println("Moved 95 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn5:
						if (turnInPlace(90)) {
							resetEncoders();
							autoStep = Step.Straight7;
							System.out.println("Turned in place to 90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					}
						if (positionSelected == centerScaleScale) {
							switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(90, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Straight8;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Straight8:
									if (driveDistanceStraight(180, -23, -0.5)) {
										resetEncoders();
										autoStep = Step.Turn7;
										System.out.println("Moved 23 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn7:
									if (turnInPlace(270)) {
										resetEncoders();
										autoStep = Step.Straight9;
										System.out.println("Turned in place to 270 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight9:
									if (driveDistanceStraight(270, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn8;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn8:
									if (turnInPlace(360)) {
										resetEncoders();
										autoStep = Step.Straight10;
										System.out.println("Turned in place to 360 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight10:
									if (driveDistanceStraight(360, 95, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn9;
										System.out.println("Moved 95 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn9:
									if (turnInPlace(450)) {
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Turned in place to 450 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Elevator3:
									if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
										autoStep = Step.Straight11;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight11:
									if (driveDistanceStraight(450, 32, 0.5)) {
										resetEncoders();
										autoStep = Step.Gripper2;
										System.out.println("Moved 32 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: _L_ has been completed");
									break;
							}
							
						}
						if (positionSelected == centerScaleSwitch) {
							if (gameData.charAt(0) == 'L') {
								switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(90, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight8;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight8:
									if (driveDistanceStraight(180, 15, 0.5)) {
										resetEncoders();
										autoStep = Step.Gripper2;
										System.out.println("Moved 15 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: LL_ has been completed");
									break;
								}
							}
							if (gameData.charAt(0) == 'R') {
								switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(90, 214, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight8;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight8:
									if (driveDistanceStraight(180, 15, 0.5)) {
										resetEncoders();
										autoStep = Step.Gripper2;
										System.out.println("Moved 15 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: RL_ has been completed");
									break;
								}
								
							}
						}
				}
				if (gameData.charAt(1) == 'R') {
					switch (autoStep) {
					case Turn1:
						if (turnInPlace(90)) {
							resetEncoders();
							autoStep = Step.Straight2;
							System.out.println("Turned in place to -90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight2:
						if (driveDistanceStraight(90, 125, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn2;
							System.out.println("Moved 125 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn2:
						if (turnInPlace(0)) {
							resetEncoders();
							autoStep = Step.Straight3;
							System.out.println("Turned in place to 0 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight3:
						if (driveDistanceStraight(0, 255.5, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn3;
							System.out.println("Moved 255.5 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn3:
						if (turnInPlace(-90)) {
							resetEncoders();
							autoStep = Step.Straight3;
							System.out.println("Turned in place to 90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight4:
						if (driveDistanceStraight(-90, 26, 0.5)) {
							resetEncoders();
							autoStep = Step.Elevator1;
							System.out.println("Moved 26 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Elevator1:
						if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
							autoStep = Step.Gripper1;
							System.out.println("Elevator Ascended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Gripper1:
						if (System.currentTimeMillis() - timerStart >= 3000) {
							autoStep = Step.Elevator2;
							System.out.println("Gripper Outed");
							timerStart = System.currentTimeMillis();
						} else {
							gripperGroup.set(1);
						}
						break;
					case Elevator2:
						if (elevatorDescend(0, 0.8)) {
							autoStep = Step.Straight5;
							System.out.println("Elevator Descended");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight5:
						if (driveDistanceStraight(-90, -32, -0.5)) {
							resetEncoders();
							autoStep = Step.Turn4;
							System.out.println("Moved 32 inches backwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn4:
						if (turnInPlace(-180)) {
							resetEncoders();
							autoStep = Step.Straight6;
							System.out.println("Turned in place to 180 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Straight6:
						if (driveDistanceStraight(-180, 95, 0.5)) {
							resetEncoders();
							autoStep = Step.Turn5;
							System.out.println("Moved 95 inches forwards");
							timerStart = System.currentTimeMillis();
						}
						break;
					case Turn5:
						if (turnInPlace(-90)) {
							resetEncoders();
							autoStep = Step.Straight7;
							System.out.println("Turned in place to 90 degrees");
							timerStart = System.currentTimeMillis();
						}
						break;
					}
						if (positionSelected == centerScaleScale) {
							switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(-90, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(-180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(-180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Straight8;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Straight8:
									if (driveDistanceStraight(-180, -23, -0.5)) {
										resetEncoders();
										autoStep = Step.Turn7;
										System.out.println("Moved 23 inches backwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn7:
									if (turnInPlace(-270)) {
										resetEncoders();
										autoStep = Step.Straight9;
										System.out.println("Turned in place to 270 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight9:
									if (driveDistanceStraight(-270, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn8;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn8:
									if (turnInPlace(-360)) {
										resetEncoders();
										autoStep = Step.Straight10;
										System.out.println("Turned in place to 360 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight10:
									if (driveDistanceStraight(-360, 95, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn9;
										System.out.println("Moved 95 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn9:
									if (turnInPlace(-450)) {
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Turned in place to 450 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Elevator3:
									if (elevatorAscend(20, 0.8)) { //TODO Set to maximum elevator height
										autoStep = Step.Straight11;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight11:
									if (driveDistanceStraight(-450, 32, 0.5)) {
										resetEncoders();
										autoStep = Step.Gripper2;
										System.out.println("Moved 32 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: _L_ has been completed");
									break;
							}
							
						}
						if (positionSelected == centerScaleSwitch) {
							if (gameData.charAt(0) == 'R') {
								switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(-90, 60, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(-180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(-180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight8;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight8:
									if (driveDistanceStraight(-180, 15, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 15 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: RR_ has been completed");
									break;
								}
							}
							if (gameData.charAt(0) == 'L') {
								switch (autoStep) {
								case Straight7:
									if (driveDistanceStraight(-90, 214, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 60 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Turn6:
									if (turnInPlace(-180)) {
										resetEncoders();
										autoStep = Step.GripStraight1;
										System.out.println("Turned in place to 180 degrees");
										timerStart = System.currentTimeMillis();
									}
									break;
								case GripStraight1:
									if (driveDistanceStraight(-180, 23, 0.3)){
										resetEncoders();
										autoStep = Step.Elevator3;
										System.out.println("Gripper Activated and moved straight");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(-1);
									}
									break;
								case Elevator3:
									if (elevatorAscend(50, 0.8)) {
										autoStep = Step.Straight8;
										System.out.println("Elevator Ascended");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Straight8:
									if (driveDistanceStraight(-180, 15, 0.5)) {
										resetEncoders();
										autoStep = Step.Turn6;
										System.out.println("Moved 15 inches forwards");
										timerStart = System.currentTimeMillis();
									}
									break;
								case Gripper2:
									if (System.currentTimeMillis() - timerStart >= 3000) {
										autoStep = Step.Done;
										System.out.println("Gripper Outed");
										timerStart = System.currentTimeMillis();
									} else {
										gripperGroup.set(1);
									}
									break;
								case Done:
									System.out.println("Auto center, ScaleSwitch, gameData: LR_ has been completed");
									break;
								}
								
							}
						}
				}
			}
		updateSmartDashboard();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		if (controlInvert == isInverted.FALSE) {
			chassis.arcadeDrive(driver.getX(), -driver.getY());
		} 
		else if (controlInvert == isInverted.TRUE) {
			chassis.arcadeDrive(driver.getX(), driver.getY());
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
		
		SmartDashboard.putNumber("Ultrasonic Yellow Distance (cm):", getUltrasonicYellowDistance());
		SmartDashboard.putNumber("Ultrasonic Black Distance (cm):", getUltrasonicBlackDistance());
		
		double distance = getDistance();
		if (bButton){
			rotationPID.setEnabled(false);
		} 
		else if (aButton) {
			//turnInPlace(90);
			driveDistanceStraight(0, -15, -0.5);
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
				if (System.currentTimeMillis() - timerStart > 1000){
					System.out.println("Invert from false to true");
					controlInvert = isInverted.TRUE;
					timerStart = System.currentTimeMillis();
				}
			} if (controlInvert == isInverted.TRUE) {
				if (System.currentTimeMillis() - timerStart > 1000){
					System.out.println("Invert from true to false");
					controlInvert = isInverted.FALSE;
					timerStart = System.currentTimeMillis();
				}
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

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void disabledPeriodic(){
		updateSmartDashboard();
	}
	
	public void PIDDriveStraight() {
		rotationPID.setEnabled(true);
		rotationPID.setSetpoint(0);
		chassis.tankDrive(((0.5)+(rotationPID.get())), (-(0.5)+(rotationPID.get())));
		System.out.println("Yes");
	}
	
	public boolean turnInPlace(double setPoint) {
		if (gyroscope.getAngle() >= (setPoint - 2) && gyroscope.getAngle() <= (setPoint + 2)) {
			rotationPID.setEnabled(false);
			resetEncoders();
			return true;
		} else {
			rotationPID.setSetpoint(setPoint);
			rotationPID.setEnabled(true);
			leftChassis.set(rotationPID.get());
			rightChassis.set(-rotationPID.get());
			return false;
		}
		
	}
	
	public boolean driveDistanceStraight(double heading, double distance, double speed) {
		System.out.println("driveDistanceStraight()");
		if (distance > 0) { //then it will move forward
			if (getDistance() > distance){
				System.out.println("current distance end");
				stop();
				return true;
			}
			else {
				driveStraight(heading, speed);
				return false;
			}
		}
		if (distance < 0) { //then it will move backward
			if (-getDistance() > -distance){
				System.out.println("current distance end");
				stop();
				return true;
			}
			else {
				driveStraight(heading, speed);
				return false;
			}
		}
		else {
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
			return true;
		}
	}
	
	public void squareDrive() {
		switch (squareStep){
		case 0:
			resetEncoders();
			rotationPID.setEnabled(false);
			driveDistanceStraight(0, 30, 0.4);
			squareStep = 1;
			break;
		case 1:
			turnInPlace(90);
			squareStep = 0;
			break;
		default:
			System.out.println("The squareDrive() function could not find a valid squareStep int");
		}
	}
	
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
		double currentAngle = gyroscope.getAngle()%360.0;
		double error = heading - currentAngle;
		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;
		if (error < 0) {
			// turn left
			leftSpeed += error * Kp;
		}
		else {
			// turn right
			rightSpeed -= error * Kp;
		}
	
		// set the motors based on the inputted speed
		leftChassis.set(leftSpeed);
		rightChassis.set(rightSpeed);
	}
	
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
		
	}
}