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
<<<<<<< HEAD
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

	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	final static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 182.13;
	
	// Smartdashboard Chooser object for Auto modes
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, gripper, elevatorOne, elevatorTwo, climberOne, climberTwo, gripperOne, gripperTwo;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, elevatorGroup, climberGroup, gripperGroup;
	
	// DifferentialDrive replaces the RobotDrive Class from previous years
	DifferentialDrive chassis;
	
	// Joystick Definitions
	Joystick driver;
	Joystick operator;
	
	//Limit Switch
	DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber, limitSwitchGripper;
	
	// Boolean for buttons
	boolean aButton, bButton, yButton, xButton, leftBumper, rightBumper, start, select, leftThumbPush, rightThumbPush, aButtonOp, bButtonOp, xButtonOp, yButtonOp;
	
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
		
		//Defines limit switch ports
		limitSwitchUpElevator = new DigitalInput(4);
		limitSwitchDownElevator =  new DigitalInput(5);
		limitSwitchUpClimber = new DigitalInput(6);
		limitSwitchDownClimber = new DigitalInput(7);
		limitSwitchGripper = new DigitalInput(8);
		
		// Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		climberOne = new Spark(4);
		climberTwo = new Spark(5);
		elevatorOne = new Spark(6);
		elevatorTwo = new Spark(7);
		climberOne = new Spark(4);
		climberTwo = new Spark(5);
		gripperOne = new Spark(8);
		gripperTwo = new Spark(9);
		
		//Inverting Sparks
		//elevatorOne.setInverted(true);
		gripperTwo.setInverted(true);
		
		// Defines Joystick ports
		driver = new Joystick(0);
		operator = new Joystick(1);

		//Creating the Joystick object
		driver = new Joystick(0);

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
		//leftElevator = new Encoder(4, 5);
		//rightElevator = new Encoder(6, 7);
		
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
>>>>>>> rz
	
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
	/*public void autonomousPeriodic() {
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
		aButtonOp = operator.getRawButton(1);
		bButtonOp = operator.getRawButton(2);
		xButtonOp = operator.getRawButton(3);
		yButtonOp = operator.getRawButton(4);
		leftBumper = driver.getRawButton(5);
		rightBumper = driver.getRawButton(6);
		select = driver.getRawButton(7);
		start = driver.getRawButton(8);
		leftThumbPush = driver.getRawButton(9);
		rightThumbPush = driver.getRawButton(10);
		
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
	
	// This is the function that allows the robot to drive straight no matter what
	// It automatically corrects itself and stays locked onto the set angle
	private void driveStraight(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle()%360.0;
		//System.out.println("Gyroscope Angle: " + gyroscope.getAngle());
		//System.out.println("Gyroscope Angle % 360.0: " + currentAngle);
		//System.out.println("driveStraight");
		double error = heading - currentAngle;
		//rotationPID.setEnabled(true);
		//rotationPID.setSetpoint(0);
		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;

		//		This code can make the robot turn very 
		//		quickly if it is not pointed close to the
		//		correct direction.  I bet this is the 
		//		problem you are experiencing.
		//		I think if you correct the state machine
		//		if statements above, you will be able to 
		//		control the turning speed.
		
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
		
		SmartDashboard.putNumber("Robot Speed", robotSpeed());

	
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
