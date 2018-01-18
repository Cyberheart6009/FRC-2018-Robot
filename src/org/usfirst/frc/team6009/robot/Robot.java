/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */                             
/*                                Stephen Clark                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6009.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//New Imports
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
			
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	SpeedController leftFront, leftBack, rightFront, rightBack, elevator, climber, gripper;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis;
	
	DifferentialDrive chassis;
	
	Joystick driver;
	
	Joystick operator;
	
	DigitalInput limitSwitch;
	
	boolean aButton, bButton, xButton, yButton, startButton, selectButton, upButton, downButton, lbumperButton, rbumperButton;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		leftChassis = new SpeedControllerGroup(leftFront, leftBack);
		rightChassis = new SpeedControllerGroup(rightFront, rightBack);
		
		rightChassis.setInverted(true);
		
		chassis = new DifferentialDrive(leftChassis, rightChassis);
		
		operator = new Joystick(0);
		driver = new Joystick(1);
		
		
		//These are the motors
		leftFront = new Spark(0);
		leftBack = new Spark(1);
		rightFront = new Spark(2);
		rightBack = new Spark(3);
		climber = new Spark(4);
		elevator = new Spark(5);
		gripper = new Spark(6);
		limitSwitch = new DigitalInput(0);
		
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
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
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
		
	//this makes it so that when the limit switch is pressed, the elevator stops (I think we did this right)
	while (limitSwitch.get()) {
		if ()
		}
	
	
		
	aButton = driver.getRawButton(1);
	bButton = driver.getRawButton(2);
	xButton = driver.getRawButton(3);
	yButton = driver.getRawButton(4);
	lbumperButton = driver.getRawButton(5);
	rbumperButton = driver.getRawButton(6);
	selectButton = driver.getRawButton(7);
	startButton = driver.getRawButton(8);
	
	chassis.arcadeDrive((driver.getX()), -(driver.getY()));
	
	elevator.set(operator.getRawAxis(5));
	
	if (aButton == true) {
		climber.set(1);
	}
	else {
		climber.set(0);
	}
	
	
	if (bButton == true) {
		gripper.set(1);
	}
	else if (xButton == true) {
		gripper.set(-1);
	}
	else {
		gripper.set(0);
	}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}