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
	
	// Constant defining encoder turns per inch of robot travel
	final static double ENCODER_COUNTS_PER_INCH = 13.49;
	//Constant defining encoder turn for the elevator lifting
	final static double ENCODER_COUNTS_PER_INCH_HEIGHT = 182.13;
	//ENCODER_COUNTS_PER_INCH_HEIGHT = 213.9
	
	// Auto Modes Setup
	String gameData;
	
	//Starting positions
	SendableChooser<String> positionChooser;
	final String left = "left";
	final String right = "right";
	final String center = "center";
	
	//what the robot does
	SendableChooser<String> movementChooser;
	final String Switch = "Switch";
	final String SwitchSwitch = "SwitchSwitch";
	final String Scale = "Scale";
	final String ScaleSwitch = "ScaleSwitch";
	final String SwitchScale = "SwitchScale";
	final String Portal = "Portal";
	
    String positionSelected;
	String movementSelected;
	
	//auto cases
		public enum Step { Straight, Turn, Straight2, Turn2, Straight3, CubeOut, CubeIn, CubeOut2,  Done }
		public Step autoStep = Step.Straight;
		public long timerStart;

	//Limit Switches
	DigitalInput limitSwitchUpElevator, limitSwitchDownElevator, limitSwitchUpClimber, limitSwitchDownClimber, limitSwitchGripper;

    SpeedController leftFront, leftBack, rightFront, rightBack, gripper1, gripper2, elevator1, elevator2, climber1, climber2, PIDSpeed;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis, elevator, gripper;
	
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
		//Chooser for the starting position
		positionChooser = new SendableChooser<String>();
		positionChooser.addObject("left", left);
		positionChooser.addObject("right", right);
		positionChooser.addObject("center", center);
		
		//Chooser for the movement of the robot
		movementChooser = new SendableChooser<String>();
		movementChooser.addObject("Switch", Switch);
		movementChooser.addObject("SwitchSwitch", SwitchSwitch);
		movementChooser.addObject("Scale", Scale);
		movementChooser.addObject("ScaleSwitch", ScaleSwitch);
		movementChooser.addObject("SwitchScale", SwitchScale);
		movementChooser.addObject("Portal", Portal);
		
		//Display these choices in whatever interface we are using
		SmartDashboard.putData("Robot Position", positionChooser);
		SmartDashboard.putData("Movement Type", movementChooser);
		
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
		
		elevator = new SpeedControllerGroup(elevator1, elevator2);
		gripper = new SpeedControllerGroup(gripper1, gripper2);
		
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
		autoStep = Step.Straight;
		gyroscope.reset();	}
	

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double distance = getDistance();
		double height = getElevatorheight();
		if (positionSelected.equalsIgnoreCase(left)) {
			if (movementSelected.equalsIgnoreCase(Switch)) {
				
			} else if (movementSelected.equalsIgnoreCase(SwitchSwitch)) {
				if (gameData.length() > 0) {
					if (gameData.charAt(0) == 'L') {
						System.out.println("leftSwitchSwitch1/2");
						switch (autoStep) {
						case Straight:
							if (distance < 220) {
								driveStraight(0, 0.4);
							}
							else {
								stop();
								autoStep = Step.Turn;
							}
							break;
						case Turn: 
							if (turnInPlace(90)) {
							  	resetEncoders();
								rotationPID.setEnabled(false);
								autoStep = Step.Straight;
							}
							break;
						case Straight2:
							if (distance < 50) {
								driveStraight(90, 0.4);
							}
							else {
								stop();
								autoStep = Step.Turn2;
							}
							break;
						case Turn2:
							if (turnInPlace(180)) {
								resetEncoders();
								rotationPID.setEnabled(false);
								autoStep = Step.CubeOut;
							}
							break;
						case CubeOut:
							if (height < 40) {
								elevator.set(0.3);
							} else {
								if (!limitSwitchGripper.get()) {
									completeStop();
									autoStep = Step.CubeIn;
								} else {
									gripper.set(0.5);
								}
							}
							break;
						case CubeIn:
							if (distance < 24){
								driveStraight(180, 0.4);
								gripper.set(-0.5);
							}
							else {
								completeStop();
								autoStep = Step.CubeOut2;
							}
							break;			
						case CubeOut2:
							if (height < 40) {
								elevator.set(0.3);
							} else {
								if (!limitSwitchGripper.get()) {
									completeStop();
									autoStep = Step.Done;
								} else {
									gripper.set(0.5);
								}
							}
							break;
						case Done:
							completeStop();
							break;
						}
					} else {
						System.out.println("leftSwitchSwitch3/4");
						
					}
				}
			} else if (movementSelected.equalsIgnoreCase(Scale)) {
				
			} else if (movementSelected.equalsIgnoreCase(ScaleSwitch)) {
				
			} else if (movementSelected.equalsIgnoreCase(SwitchScale)) {
				
			} else if (movementSelected.equalsIgnoreCase(Portal)) {
				
			} else {
				completeStop();
			}
		} else if (positionSelected.equalsIgnoreCase(right)) {
			if (movementSelected.equalsIgnoreCase(Switch)) {
				
			} else if (movementSelected.equalsIgnoreCase(SwitchSwitch)) {
				if (gameData.length() > 0) {
					if (gameData.charAt(0) == 'L') {
						System.out.println("rightSwitchSwitch1/2");
					} else {
						System.out.println("rightSwitchSwitch3/4");
						switch (autoStep) {
						case Straight:
							if (distance < 220) {
								driveStraight(0, 0.4);
							}
							else {
								stop();
								autoStep = Step.Turn;
							}
							break;
						case Turn: 
							if (turnInPlace(-90)) {
							  	resetEncoders();
								rotationPID.setEnabled(false);
								autoStep = Step.Straight;
							}
							break;
						case Straight2:
							if (distance < 50) {
								driveStraight(-90, 0.4);
							}
							else {
								stop();
								autoStep = Step.Turn2;
							}
							break;
						case Turn2:
							if (turnInPlace(-180)) {
								resetEncoders();
								rotationPID.setEnabled(false);
								autoStep = Step.CubeOut;
							}
							break;
						case CubeOut:
							if (height < 40) {
								elevator.set(0.3);
							} else {
								if (!limitSwitchGripper.get()) {
									completeStop();
									autoStep = Step.CubeIn;
								} else {
									gripper.set(0.5);
								}
							}
							break;
						case CubeIn:
							if (distance < 24){
								driveStraight(-180, 0.4);
								gripper.set(-0.5);
							}
							else {
								completeStop();
								autoStep = Step.CubeOut2;
							}
							break;			
						case CubeOut2:
							if (height < 40) {
								elevator.set(0.3);
							} else {
								if (!limitSwitchGripper.get()) {
									completeStop();
									autoStep = Step.Done;
								} else {
									gripper.set(0.5);
								}
							}
							break;
						case Done:
							completeStop();
							break;
						}
					}
			} else if (movementSelected.equalsIgnoreCase(Scale)) {
				
			} else if (movementSelected.equalsIgnoreCase(ScaleSwitch)) {
				
			} else if (movementSelected.equalsIgnoreCase(SwitchScale)) {
				
			} else if (movementSelected.equalsIgnoreCase(Portal)) {
				
			} else {
				completeStop();
			}
		} else if (positionSelected.equalsIgnoreCase(center)){
			
		} else {
			completeStop();
		}
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
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
		return ((double)(leftElevator.get() + rightElevator.get()) / (ENCODER_COUNTS_PER_INCH_HEIGHT * 2));
	}
	public double getDistance(){
		return ((double)(leftEncoder.get() + rightEncoder.get()) / (ENCODER_COUNTS_PER_INCH * 2));
	}
	
	private void PIDTurn(double setpoint) {
		resetEncoders();
		rotationPID.setSetpoint(setpoint);
		rotationPID.setEnabled(true);
		leftChassis.set(rotationPID.get());
		rightChassis.set(-rotationPID.get());
	}
	
	public boolean turnInPlace(double setPoint) {
		/* FIXME Setpoint is currently required to be exact, implement a range*/
		if (gyroscope.getAngle() >= (setPoint - 2) && gyroscope.getAngle() <= (setPoint + 2)) {
			return true;
		} else {
			rotationPID.setSetpoint(setPoint);
			rotationPID.setEnabled(true);
			leftChassis.set(rotationPID.get());
			rightChassis.set(-rotationPID.get());
			return false;
		}
		
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
	}
	private void stopElevator() {
		elevator1.set(0);
		elevator2.set(0);
	}
	private void stopGripper() {
		gripper1.set(0);
		gripper2.set(0);
	}
	private void completeStop() {
		stop();
		stopElevator();
		stopGripper();
	}
	
	@Override
	   /*This function is invoked periodically by the PID Controller, */
	  /* based upon navX-MXP yaw angle input and PID Coefficients.    */
	  public void pidWrite(double output) {
	      
	  }
	
}