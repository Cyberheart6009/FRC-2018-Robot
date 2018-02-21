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
	// Creating Movement Chooser and its choices
	SendableChooser<String> movementChooser;
	private static final String switchSwitch = "switchSwitch";
	private static final String switchScale = "switchScale";
	private static final String scaleSwitch = "scaleSwitch";
	private static final String scaleScale = "scaleScale";
	String positionSelected;
	String movementSelected;
	
	//auto cases
	// Turn2, Turn3, Turn4, Straight2, Straight3, Straight4
	public enum Step { Straight, Straight2, Straight3, Straight4, Turn, Turn1, Turn2, Turn3, Turn4, ElevatorUp1, GripperOut1 }
	public Step autoStep = Step.Straight;
	public long timerStart;

	//Variables
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	
	// SpeedController Object creations - Define all names of motors here
	SpeedController leftFront, leftBack, rightFront, rightBack, climber1, climber2, elevator1, elevator2, gripper1, gripper2;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, elevatorGroup, climberGroup, gripperGroup;
	
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
	//AHRS gyroscope;
	
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
		// Create Movement Types
		movementChooser = new SendableChooser<String>();
		movementChooser.addObject("Switch 2x", switchSwitch);
		movementChooser.addObject("Switch then Scale", switchScale);
		movementChooser.addObject("Scale then Switch", scaleSwitch);
		movementChooser.addObject("Scale 2x", scaleScale);
		//Display these choices in whatever interface we are using
		SmartDashboard.putData("Robot Position", positionChooser);
		SmartDashboard.putData("Movement Type", movementChooser);

		
		// Defines all the ports of each of the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		climber1 = new Spark(4);
		climber2 = new Spark(5);
		elevator1 = new Spark(6);
		elevator2 = new Spark(7);
		gripper1 = new Spark(8);
		gripper2 = new Spark(9);
		
		driver = new Joystick(0);
		
		// Defines the left and right SpeedControllerGroups for our DifferentialDrive class
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		elevatorGroup = new SpeedControllerGroup(elevator1, elevator2);
		climberGroup = new SpeedControllerGroup(climber1, climber2);
		gripperGroup = new SpeedControllerGroup(gripper1, gripper2);
		
		// Inverts the right side of the drive train to account for the motors being physically flipped
		leftChassis.setInverted(true);
		
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
		//gyroscope = new AHRS(SPI.Port.kMXP);
		gyroscope.calibrate();
		
		rotationPID = new PIDController(Kp, Ki, Kd, gyroscope, this);
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
		gyroscope.reset();
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double distance = getDistance();
		if (positionSelected.equalsIgnoreCase(square)) {
			System.out.println("Square Auto Is Operating");
			switch(autoStep){
				case Straight:
					if (distance < 60){
						driveStraight(23, 0.3);
						System.out.println("Going Straight");
					}
					else{
						stop();
						resetEncoders();
						autoStep = Step.Turn;
						System.out.println("Encoders Reset!");
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
					break;
			}
			/*switch (autoStep) {
			case Straight:
				resetEncoders();
				driveStraight(0, 0.3);
				if (distance > 10) {
					stop();
				}
				timerStart = System.currentTimeMillis();
				autoStep = Step.Turn;
				break;
			case Turn:
				
				rotationPID.setSetpoint(90);
				rotationPID.setEnabled(true);
				rightChassis.set(-(rotationPID.get()));
				leftChassis.set((rotationPID.get()));
				gyroscope.reset();
				
				timerStart = System.currentTimeMillis();
				autoStep = Step.Done;
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
			}*/
		}
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
				case Straight:
					resetEncoders();
					driveStraight(0, 0.4);
					if (distance > 70) {
						stop();
					}
					timerStart = System.currentTimeMillis();
					autoStep = Step.Turn;
					break;
				}
				if (gameData.charAt(0) == 'L') {
					// TODO Insert Code to pick up box and move to the left side of the scale
					switch (autoStep) {
					case Turn:
						rotationPID.setSetpoint(-90);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight2;
						break;
					case Straight2:
						resetEncoders();
						driveStraight(0, 0.4);
						if (distance >= 59) {
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.Turn2;
						break;
					case Turn2:
						rotationPID.setSetpoint(0);
						rotationPID.setEnabled(true);
						leftChassis.set(rotationPID.get());
						rightChassis.set(-rotationPID.get());
						timerStart = System.currentTimeMillis();
						autoStep = Step.Straight3;
						break;
					case Straight3:
						int height1 = 0; //remove this after an actual elevator height equation is created
						resetEncoders();
						driveStraight(0, 0.4);
						if (distance >= 70) {
							stop();
						}
						timerStart = System.currentTimeMillis();
						autoStep = Step.Turn;
						elevatorGroup.set(0.3);
						if (height1 == 20) {
							elevatorGroup.set(0);
						}
						break;
					case GripperOut1:
						gripperGroup.set(0.3);
					default:
						System.out.println("haha");
						break;
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
		chassis.arcadeDrive(driver.getX(), driver.getY());
		
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
			squareDrive();
		}
		if(xButton){
			gyroscope.reset();
			resetEncoders();
		}
		if(yButton){
			driveStraight(0, 0.3);
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
	
	public void turnInPlace(double setPoint) {
		gyroscope.reset();
		rotationPID.setSetpoint(setPoint);
		rotationPID.setEnabled(true);
		rightChassis.set(-(rotationPID.get()));
		leftChassis.set((rotationPID.get()));
	}
	
	public void driveDistanceStraight(double distance, double speed) {
		double currentDistance = getDistance();
		driveStraight(0, 0.4);
		System.out.println("driveDistanceStraight()");
		if (currentDistance > distance){
			System.out.println("current distance end");
			stop();
			resetEncoders();
			return;
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
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle()%360.0;
		System.out.println("Gyroscope Angle: " + gyroscope.getAngle());
		System.out.println("Gyroscope Angle % 360.0: " + currentAngle);
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
	
	private boolean turnRight(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(0.35);
		leftFront.set(0.35);
		rightBack.set(-0.35);
		rightFront.set(-0.35);

		System.out.println("Turning Right");
		
		double currentAngle = gyroscope.getAngle();
		if (currentAngle >= targetAngle - 10){
			System.out.println("Stopped Turning Right");
			return true;
		}
		return false;
	}
	
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}
}


//Legacy Autonomous & Katerinas Square Auto
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