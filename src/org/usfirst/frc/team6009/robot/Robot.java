/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//|**************************************************************|
//|				Cyberheart 2018 First Power Up 					 |
//|																 |
//|					Testing 2018								 |
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
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

import org.spectrum3847.RIOdroid.RIOadb;
import org.spectrum3847.RIOdroid.RIOdroid;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {
	
	// Auto Modes Setup
	private static final String centerSwitch = "Center Switch";
	private static final String DriveSTRAIGHT = "STRAIGHT Drive";
	private static final String RightSwitch = "Right Switch";
	private static final String LeftSwitch = "Left Switch";	
	private static final String RightSwitchTest = "Right Switch";
	private static final String LeftSwitchTest = "Left Switch";
	private static final String DoubleRightSwitch = "Double Right Switch";
	private static final String DoubleLeftSwitch = "Double Left Switch";
	private static final String LeftDoubleScale = "Left Double Scale";
	private static final String RightDoubleScale = "Right Double Scale";
	private static final String LeftAngleScale = "Left Angle Scale";
	private static final String RightAngleScale = "Right Angle Scale";
	private static final String LeftOppositeSideScale = "LeftOppositeSideScale";
	private static final String RightOppositeSideScale = "RightOppositeSideScale";
	private static final String centerOppositeSwitch = "Center Switch";
	private static final String testAuto = "Test Mode";
	private static final String DanielHAuto = "Daniel H. AutoMode";
	private static final String RightSwitch2 = "Right Switch 2";
	private static final String LeftSwitch2 = "Left Switch 2";
	private String autoSelected;
	
	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
	final static double DEGREES_PER_PIXEL = 0.100;
	double currentSpeed = 0.3;
	double roboSpeed;
	double oldEncoderCounts = 0;
	long old_time = 0;
	String box_position = "NO DATA";
	boolean initializeADB = false;
	String gameData;
	long timerStart;
	boolean TP_Active, TP_Trigger = false;
	double angle = 0;
	double old_angle = 0;
	int deccel_counter;
	
	// Smartdashboard Chooser object for Auto modes
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, gripper, elevatorOne, elevatorTwo, climberOne, climberTwo, TP_motor;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, climberGroup, elevator;
	
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
	double kP = 0.04;
	double smoothKp = 0.03;
	
	//Auto variables
	public enum speedStep{ACCEL, COAST, DECCEL};
	public speedStep driveStep = speedStep.ACCEL;
	public enum Step {STRAIGHT, SHORT_STRAIGHT, SHORTER_STRAIGHT, STRAIGHT1CENTER, STRAIGHT2, TURN1CENTER,INTAKEDRIVE, STRAIGHT2CENTER, TURN2CENTER, STRAIGHT3CENTER, STRAIGHT3, TURN, TURN2, TURN3, TURN4, LIFT, LIFT2, DESCEND, TRACK, LAUNCH, LAUNCH2, DONE, MYFIRSTSTRAIGHT, MYLEFTTURN, MYSECONDSTRAIGHT, MYTHIRDSTRAIGHT, MFOURTHSTRAIGHT, MYDROPCUBE};
	public Step autoStep = Step.STRAIGHT;
	
	//Serial Port Communication
	//public static final SerialPort.Port kUSB2;
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Adds all of our previously created auto modes into the smartdashboard chooser
		m_chooser.addObject("Center Switch", centerSwitch);
		m_chooser.addDefault("My Auto", DriveSTRAIGHT);
		m_chooser.addObject("Right Switch", RightSwitch);
		m_chooser.addObject("Left Switch", LeftSwitch);
		m_chooser.addObject("Double Left Switch", DoubleLeftSwitch);
		m_chooser.addObject("Double Right Switch", DoubleRightSwitch);
		m_chooser.addObject("Left Double Scale", LeftDoubleScale);
		m_chooser.addObject("Right Double Scale", RightDoubleScale);
		m_chooser.addObject("Left Angle Scale", LeftAngleScale);
		m_chooser.addObject("Right Angle Scale", RightAngleScale);
		m_chooser.addObject("Left Opposite Side Scale", LeftOppositeSideScale);
		m_chooser.addObject("RightOppositeSideScale", RightOppositeSideScale);
		m_chooser.addObject("Center Scale", centerOppositeSwitch);
		m_chooser.addObject("Test Auto", testAuto);
		m_chooser.addObject("Right Switch 2", RightSwitch2);
		m_chooser.addobject("Left Switch 2", LeftSwitch2);
		m_chooser.addObject("Daniel H. AutoMode", DanielHAuto);)
		SmartDashboard.putData("Auto choices", m_chooser);
		//final SerialPort.Port kUSB2;
		
		//SerialPort name = new SerialPort(9600,);

		
		// Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		climberOne = new Spark(4);
		climberTwo = new Spark(5);
		elevatorOne = new Spark(6);
		elevatorTwo = new Spark(7);
		TP_motor = new Spark(8);
		gripper = new Spark(9);
		
		//Inverting Sparks
		//climberOne.setInverted(true);
		

		// Defines Joystick ports
		driver = new Joystick(0);
		operator = new Joystick(1);

		cubeSwitch = new DigitalInput(4);

		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		climberGroup = new SpeedControllerGroup(climberOne, climberTwo);
		elevator = new SpeedControllerGroup(elevatorOne, elevatorTwo);
		
		climberGroup.setInverted(true);
		
		elevator.setInverted(true);
		gripper.setInverted(true);
		
		// Inverts the right side of the drive train to account for the motors being physically flipped
		rightChassis.setInverted(true);
		//TP_motor.setInverted(true);
		
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
		/*if ((autoSelected.equalsIgnoreCase(LeftDoubleScale) || (autoSelected.equalsIgnoreCase(RightDoubleScale)))){
			if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
				autoSelected = RightDoubleScale;
			}
			else if (gameData.charAt(1) == 'L' && autoSelected == RightDoubleScale){
				autoSelected = RightOppositeSideScale;
			}
			if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
				autoSelected = LeftDoubleScale;
			}
			else if (gameData.charAt(1) == 'R' && autoSelected == LeftDoubleScale){
				autoSelected = LeftOppositeSideScale;
			}
			else{
				autoSelected = autoSelected;
			}
		}*/
		if ((autoSelected.equalsIgnoreCase(RightSwitch)) || autoSelected.equalsIgnoreCase(LeftSwitch)) {
			autoStep = Step.STRAIGHT;
		}
		if ((autoSelected.equalsIgnoreCase(centerSwitch))) {
			autoStep = Step.STRAIGHT1CENTER;
		}
		else{
			autoStep = Step.STRAIGHT;
		}
		
		resetEncoders();
		elevatorEncoder.reset();
		gyroscope.reset();
		deccel_counter = 0;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		updateSmartDashboard();
		/*
		if ((autoSelected.equalsIgnoreCase(LeftDoubleScale) || (autoSelected.equalsIgnoreCase(RightDoubleScale)))){
			if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
				autoSelected = RightDoubleScale;
			}
			else if (gameData.charAt(1) == 'L' && autoSelected == RightDoubleScale){
				autoSelected = RightOppositeSideScale;
			}
			if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
				autoSelected = LeftDoubleScale;
			}
			else if (gameData.charAt(1) == 'R' && autoSelected == LeftDoubleScale){
				autoSelected = LeftOppositeSideScale;
			}
			else{
				autoSelected = autoSelected;
			}
		}*/
		
		if (autoSelected.equalsIgnoreCase(DanielHAuto)) {
			autoStep = Step.MYFIRSTSTRAIGHT
			int x = 0
			switch (autoStep) {
			case MYFIRSTSTRAIGHT:
				smoothDrive(1, 1, 240)
				autoStep = Step.MYLEFTTURN				
				break;
			case MYLEFTTURN:
				turnLeft(-90)
				if (x = 0) {
					autoStep = Step.MYSECONDSTRAIGHT
					x = 1
				} 
				else if (x = 1) {
					autoStep = Step.MYTHIRDSTRAIGHT
					x = 2
				}
				else {
					autoStep = Step.MYFOURTHSTRAIGHT
							
				}
				break;
			case MYSECONDSTRAIGHT:
				smoothDrive(1, 1, 168)
				autoStep = Step.MYLEFTTURN
				break;
			case MYTHIRDSTRAIGHT:
				smoothDrive(1, 1, 60)
				autoStep = Step.MYLEFTTURN
				break;
			case MYFOURTHSTRAIGHT:
				smoothDrive(1, 1, 24)
				autoStep = Step.MYDROPCUBE
				break;
			case MYDROPCUBE:
				if ((System.currentTimeMillis() - timerStart) < 450) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DESCEND;
				}
				break;
				
			}		
		
		if (autoSelected.equalsIgnoreCase(RightSwitch2) || autoSelected.equalsIgnoreCase(LeftSwitch2)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (distance < 463){
					driveSTRAIGHT(0, 0.5);
				}
				else{
					stop();
					autoStep = Step.TURN;
				}
				break;
			case TURN:
				if (gameData.charAt(2) == 'R' && autoSelected == RightSwitch2){
					if (turnLeft(-90)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else if(gameData.charAt(2) == 'L' && autoSelected == LeftSwitch2){
					if (turnRight(90)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else {
					stop();
					autoStep = Step.DONE;
				}
				break;
			case SHORT_STRAIGHT:
				if (autoSelected == LeftSwitch2){
					if (distance > 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				else{
					driveSTRAIGHT(90, 0.4);
				}
				}
				else {
				if (distance > 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
				stop();
				timerStart = System.currentTimeMillis();
				autoStep = Step.LAUNCH;
				}
				else{
				driveSTRAIGHT(-90, 0.4);
				}
				}
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 1000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
			}
		
		if (autoSelected == DriveSTRAIGHT) {
			if (getDistance() < 180){
				driveSTRAIGHT(0, 0.5);
				System.out.println("Going STRAIGHT");
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
				if (distance < 155){
					driveSTRAIGHT(0, 0.5);
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
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else if(gameData.charAt(0) == 'L' && autoSelected == LeftSwitch){
					if (turnRight(90)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else {
					stop();
					autoStep = Step.DONE;
				}
				break;
			case SHORT_STRAIGHT:
				if (autoSelected == LeftSwitch){
					if (distance > 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
					else{
						driveSTRAIGHT(90, 0.4);
					}
				}
				else {
					if (distance > 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
					else{
						driveSTRAIGHT(-90, 0.4);
					}
				}				
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 1000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
		if (autoSelected.equalsIgnoreCase(RightSwitchTest) || autoSelected.equalsIgnoreCase(LeftSwitchTest)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (distance < 160){
					driveSTRAIGHT(0, 0.5);
				}
				else{
					stop();
					resetEncoders();
					autoStep = Step.TURN;
				}
				if ((gameData.charAt(0) == 'L' && autoSelected == RightSwitchTest) || (gameData.charAt(0) == 'R' && autoSelected == LeftSwitchTest)) {
					if (distance < 80){
						driveSTRAIGHT(0, 0.5);
					}
					else{
						stop();
						resetEncoders();
						autoStep = Step.TURN;
					}
				}
				break;
			case TURN:
				if (autoSelected == RightSwitchTest){
					if (turnLeft(-90)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else if(autoSelected == LeftSwitchTest){
					if (turnRight(90)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else {
					stop();
					autoStep = Step.DONE;
				}
				break;
			case SHORT_STRAIGHT:
				if (autoSelected == LeftSwitchTest && gameData.charAt(0) == 'L'){
					if (distance < 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
						driveSTRAIGHT(90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}
				else if (gameData.charAt(0) == 'R' && autoSelected == RightSwitchTest){
					if (distance < 30 || ((System.currentTimeMillis() - timerStart) < 1000)){
						driveSTRAIGHT(-90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}		
				if (autoSelected == LeftSwitchTest && gameData.charAt(0) == 'R'){
					if (distance < 153){
						driveSTRAIGHT(90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.TURN2;
					}
				}
				else if (gameData.charAt(0) == 'L' && autoSelected == RightSwitchTest){
					if (distance < 153){
						driveSTRAIGHT(-90, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.TURN2;
					}
				}				
				break;
			case TURN2:
				if (autoSelected == RightSwitchTest){
					if (turnLeft(-225)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORTER_STRAIGHT;
					}
				}
				else if(autoSelected == LeftSwitchTest){
					if (turnRight(225)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORTER_STRAIGHT;
					}
				}
				else {
					stop();
					autoStep = Step.DONE;
				}
				break;
			case SHORTER_STRAIGHT:
				if (autoSelected == RightSwitchTest){
					if (distance < 30 || ((System.currentTimeMillis() - timerStart) > 1000)){
						driveSTRAIGHT(-225, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}
				else if (autoSelected == LeftSwitchTest){
					if (distance < 30 || ((System.currentTimeMillis() - timerStart) < 1000)){
						driveSTRAIGHT(225, 0.4);
					}
					else{
						stop();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LAUNCH;
					}
				}
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 2000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
		
		// FIXME: Double Scale Code - begin to hack together
		// 			Change Timed based lift / descent to encoder counts
		//			Must test to see if smooth drive distances are working
		// Should be 10 - 11 cases (11th is the bring down arms at end of auto)
		if (autoSelected.equalsIgnoreCase(RightDoubleScale) || autoSelected.equalsIgnoreCase(LeftDoubleScale)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (distance < 243){ // Was 244
						smoothDrive(-5, 1, 237);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (distance < 243){ // was 244
						smoothDrive(5, 1, 237);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				}
				else{
					if (distance < 180){
						driveSTRAIGHT(0, 0.5);
					}
					else{
						elevator.set(0);
						stop();
						autoStep = Step.DONE;
					}
				}
				break;
			case TURN:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (turnLeft(-20)) {
						stop();
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (turnRight(20)) {
						stop();
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				}
				else{
					autoStep = Step.DONE;
					}
				break;

			case LIFT:
				// lift to 127 !!!
				if (getElevatorHeight() < 170 && (System.currentTimeMillis() - timerStart) < 2500){
					elevator.set(-0.8);
				}
				else{
					stop();
					elevator.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.LAUNCH;
				}
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 450) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DESCEND;
				}
				break;
			case DESCEND:
				if (getElevatorHeight() > 5){
					elevator.set(0.7);
				}
				else{
					stop();
					elevator.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN2;
				}
				break;
			case TURN2:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (turnLeft(-156)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.INTAKEDRIVE;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (turnRight(156)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.INTAKEDRIVE;
					}
				}
				else{
					autoStep = Step.DONE;
					}
				break;
			case INTAKEDRIVE:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (distance < 42){ // Was 244
						driveSTRAIGHT(-156, 0.5);
						gripper.set(-1);
					}
					else{
						stop();
						autoStep = Step.TURN3;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (distance < 42){ // was 244
						driveSTRAIGHT(156, 0.5);
						gripper.set(-1);
					}
					else{
						stop();
						autoStep = Step.TURN3;
					}
				}
				break;
			case TURN3:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (turnRight(8)) {
						resetEncoders();
						gripper.set(0);
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (turnLeft(-8)) {
						resetEncoders();

						gripper.set(0);
						timerStart = System.currentTimeMillis();
						autoStep = Step.SHORT_STRAIGHT;
					}
				}
				else{
					autoStep = Step.DONE;
					}
				break;
			case SHORT_STRAIGHT:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (distance < 36){ // Was 244
						driveSTRAIGHT(1, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN4;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (distance < 36){ // was 244
						driveSTRAIGHT(-1, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN4;
					}
				}
				break;
			case TURN4:
				if (gameData.charAt(1) == 'R' && autoSelected == RightDoubleScale){
					if (turnLeft(-20)) {
						stop();
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT2;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftDoubleScale){
					if (turnRight(20)) {
						stop();
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT2;
					}
				}
				else{
					autoStep = Step.DONE;
					}
				break;
			case LIFT2:
				// lift to 127 !!!
				if (getElevatorHeight() < 170 && (System.currentTimeMillis() - timerStart) < 2500){
					elevator.set(-0.8);
				}
				else{
					stop();
					elevator.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.LAUNCH2;
				}
				break;
			case LAUNCH2:
				if ((System.currentTimeMillis() - timerStart) < 450) {
					gripper.set(0.8);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
		if (autoSelected.equalsIgnoreCase(RightAngleScale) || autoSelected.equalsIgnoreCase(LeftAngleScale)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (gameData.charAt(1) == 'R' && autoSelected == RightAngleScale){
					if (distance < 244){
						driveSTRAIGHT(-4, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftAngleScale){
					if (distance < 244){
						driveSTRAIGHT(4, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				}
				else{
					if (distance < 180){
						driveSTRAIGHT(0, 0.5);
					}
					else{
						stop();
						autoStep = Step.DONE;
					}
				}
				break;
			case TURN:
				if (gameData.charAt(1) == 'R' && autoSelected == RightAngleScale){
					if (turnLeft(-30)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				}
				else if (gameData.charAt(1) == 'L' && autoSelected == LeftAngleScale){
					if (turnRight(30)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				}
				else{
					autoStep = Step.DONE;
					}
				break;

			case LIFT:
				// lift to 127 !!!
				if (getElevatorHeight() < 170 && (System.currentTimeMillis() - timerStart) < 2100){
					elevator.set(-0.7);
				}
				else{
					stop();
					elevator.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.LAUNCH;
				}
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 2000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
		if (autoSelected.equalsIgnoreCase(centerSwitch)){
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT1CENTER:
				if (distance < 29){
					driveSTRAIGHT(0, 0.5);
				}
				else{
					stop();
					autoStep = Step.TURN1CENTER;
				}
				break;
			case TURN1CENTER:
				if (gameData.charAt(0) == 'R'){
					if (turnRight(90)){
						resetEncoders();
						autoStep = Step.STRAIGHT2CENTER;
					}
				}
				else{
					if (turnLeft(-90)) {
						resetEncoders();
						autoStep = Step.STRAIGHT2CENTER;
					}
				
				}
				break;
			case STRAIGHT2CENTER:
				if (gameData.charAt(0) == 'R'){
					if (distance < 49){
						driveSTRAIGHT(90, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN2CENTER;
					}
				}
				else{
					if (distance < 62){
						driveSTRAIGHT(-90, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN2CENTER;
					}
				
				}
				break;
			case TURN2CENTER:
				if (gameData.charAt(0) == 'R'){
					if (turnLeft(0)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.STRAIGHT3CENTER;
					}
				}
				else{
					if (turnRight(0)){
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.STRAIGHT3CENTER;
					}
				
				}
				break;
			case STRAIGHT3CENTER:
				if (distance < 71 && (System.currentTimeMillis() - timerStart) < 2500){
					driveSTRAIGHT(0, 0.5);
				}
				else{
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.LAUNCH;
				}
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 2000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
		return;
		}
		if (autoSelected.equalsIgnoreCase(LeftOppositeSideScale) || autoSelected.equalsIgnoreCase(RightOppositeSideScale)) {
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'L') {
					if (distance < 244){
						driveSTRAIGHT(4, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				} else if (autoSelected == RightOppositeSideScale && gameData.charAt(1) == 'R') {
					if (distance < 244){
						driveSTRAIGHT(-4, 0.5);
					}
					else{
						stop();
						autoStep = Step.TURN;
					}
				} else {
					if (distance < 207) {
						smoothDrive(0, 1, 205);
					} else {
						stop();
						autoStep = Step.TURN;
					}
				} 
				break;
			case TURN:
				if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'L') {
					if (turnRight(30)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				} else if (autoSelected == RightOppositeSideScale && gameData.charAt(1) == 'R') {
					if (turnLeft(-30)) {
						resetEncoders();
						timerStart = System.currentTimeMillis();
						autoStep = Step.LIFT;
					}
				} else if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'R') {
					if (turnRight(90)) {
						resetEncoders();
						autoStep = Step.STRAIGHT2;
					}
				} else {
					if (turnLeft(-90)) {
						resetEncoders();
						autoStep = Step.STRAIGHT2;
					}
				}
				break;
			case STRAIGHT2:
				if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'R') {
					if (distance < 233) {
						smoothDrive(90, 1, 228);
					} else {
						stop();
						autoStep = Step.TURN2;
					}
				} else if (autoSelected == RightOppositeSideScale && gameData.charAt(1) == 'L') {
					if (distance < 233) {
						smoothDrive(-90, 1, 228);
					}
				} else {
					autoStep = Step.TURN2;
				}
				break;
			case TURN2:
				if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'R') {
					if (turnLeft(0)) {
						resetEncoders();
						autoStep = Step.STRAIGHT3;
					} 
				} else if (autoSelected == RightOppositeSideScale && gameData.charAt(1) == 'L') {
					if (turnRight(0)) {
						resetEncoders();
						autoStep = Step.STRAIGHT3;
					}
				} else {
					autoStep = Step.DONE;
				}
				break;
			case STRAIGHT3:
				if (distance < 45) {
					driveSTRAIGHT(0, 0.5);
				} else {
					stop();
					autoStep = Step.TURN3;
				}
				break;
			case TURN3:
				if (autoSelected == LeftOppositeSideScale && gameData.charAt(1) == 'R') {
					if (turnLeft(-60)) {
						stop();
						resetEncoders();
						autoStep = Step.LIFT;
					} 
				} else if (autoSelected == RightOppositeSideScale && gameData.charAt(1) == 'L') {
					if (turnRight(60)) {
						stop();
						resetEncoders();
						autoStep = Step.LIFT;
					}
				} else {
					autoStep = Step.DONE;
				}
				break;
			case LIFT:
				if (getElevatorHeight() < 170 || (System.currentTimeMillis() - timerStart) < 2500){
					elevator.set(-0.8);
				}
				else{
					stop();
					elevator.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.LAUNCH;
				}
				break;
			case LAUNCH:
				if ((System.currentTimeMillis() - timerStart) < 2000) {
					gripper.set(1);
				}
				else{
					gripper.set(0);
					autoStep = Step.DONE;
				}
				break;
			case DONE:
				break;
			}
			return;
		}
		if (autoSelected.equalsIgnoreCase(testAuto)){
			if (getDistance() < 300){
				//FIXME
				// @ 1 = 12" overshoot
				// @ 0.8 = 9" overshoot
				smoothDrive(0, 0.8, 300);
			}
			else{
				stop();
				System.out.println("STOPPED");
			}
		}
		return;
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		tipPrevention();
		chassis.arcadeDrive(driver.getX(), -driver.getY());
		System.out.println("Motor Power: " + driver.getY());
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
		} */
		if (aButton){
			elevatorEncoder.reset();
		}/*
		else if (bButton){
			TP_motor.set(-1);
		}
		else{
			elevator.set(0);
		}*/
		
		// OPERATOR CONTROLS
		if (aButtonOp) {
			gripper.set(1);
		}
		else if (yButtonOp) {
			gripper.set(0.5);
		}
		else if (bButtonOp) {
			gripper.set(-1);
		}
		else {
			gripper.set(0);
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
		roboSpeed = ((getDistance() - oldEncoderCounts)/(System.currentTimeMillis() - old_time)) * 0.0254;
		old_time = System.currentTimeMillis();
		oldEncoderCounts = getDistance();
		return (double) roboSpeed;
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
		angle = gyroscope.getPitch();
		//System.out.println(angle); - Debug line
		
		if ((System.currentTimeMillis() - timerStart) < 450){
			TP_motor.set(0.6);
		}
		else if (old_angle > 0 && angle <= 0){
			timerStart = System.currentTimeMillis();
		}
		else if (angle > 0 && angle < 70){
			TP_motor.set(-1.0);
		}
		else if(angle > 75){
			TP_motor.set(0);
		}
		else{
			TP_motor.set(0);
		}
		old_angle = angle;
		
	}
	
	private void driveSTRAIGHT(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = (gyroscope.getAngle()%360.0);
		//System.out.println("driveSTRAIGHT");
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
	
	private void smoothDrive(double heading, double targetSpeed, double targetDistance){
		System.out.println("Current Speed: " + currentSpeed);
		if (getDistance() < 12 && currentSpeed < targetSpeed){
			driveStep = speedStep.ACCEL;
			System.out.println("Accelerating");
		}// was 58 ((78*(Math.pow((Math.pow(targetSpeed, 10)), 1.0/9))) + 5)
		else if(getDistance() > targetDistance - (50*targetSpeed)){
			driveStep = speedStep.DECCEL;
			System.out.println("Deccelerating");
		}
		else{
			driveStep = speedStep.COAST;
			System.out.println("Coast");
		}
		switch (driveStep) {
		case ACCEL:
			currentSpeed += 0.02;
			driveSTRAIGHT(heading, currentSpeed);
			break;
		case DECCEL:
			if (currentSpeed > 0.5){
				currentSpeed -= 0.05;
			}
			else {
				//FIXME
				if (deccel_counter <= 40 - 10 ){
					deccel_counter++;
					//at 1 speed it was 0.25
					currentSpeed = 0.27;
				}
				else{
					currentSpeed = 0.4; // raise to 0.5?
				}
			}
			driveSTRAIGHT(heading, currentSpeed);
			break;
		case COAST:
			driveSTRAIGHT(heading, targetSpeed);
			break;
		}		
	}
	
	private void smootherDrive(double heading, double targetSpeed, double targetDistance) {
		System.out.println("Current Speed: " + currentSpeed);
		double error = targetDistance - getDistance();
		if (getDistance() < 12 && currentSpeed < targetSpeed){
			driveStep = speedStep.ACCEL;
			System.out.println("Accelerating");
		}
		else if(getDistance() > targetDistance - 24){
			driveStep = speedStep.DECCEL;
			System.out.println("Deccelerating");
		}
		else{
			driveStep = speedStep.COAST;
			System.out.println("Coast");
		}
		switch (driveStep) {
		case ACCEL:
			currentSpeed += 0.02;
			driveSTRAIGHT(heading, currentSpeed);
			break;
		case DECCEL:
			currentSpeed -= smoothKp * error;
			driveSTRAIGHT(heading, currentSpeed);
			break;
		case COAST:
			driveSTRAIGHT(heading, targetSpeed);
			break;
		}		
	}
	
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}
	
	private boolean turnRight(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(0.55);
		leftFront.set(0.55);
		rightBack.set(0.55);
		rightFront.set(0.55);

		System.out.println("TURNing Right");
		
		double currentAngle = gyroscope.getAngle();
		if (currentAngle >= targetAngle - 2){
			System.out.println("Stopped TURNing Right");
			return true;
		}
		return false;
	}
	private boolean turnLeft(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(-0.55);
		leftFront.set(-0.55);
		rightBack.set(-0.55);
		rightFront.set(-0.55);

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