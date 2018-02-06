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
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;


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
	
	SpeedController leftFront, leftBack, rightFront, rightBack;
	
	// Speed controller group used for new differential drive class
	SpeedControllerGroup leftChassis, rightChassis;
	
	DifferentialDrive chassis;
	
	Joystick driver;
	
	Servo leftServo, rightServo;

	Ultrasonic ultra = new Ultrasonic(1,1); // creates the ultra object andassigns ultra to be an ultrasonic sensor which uses DigitalOutput 1 for 
    										// the echo pulse and DigitalInput 1 for the trigger pulse

	DigitalInput upperLimitSwitch, lowerLimitSwitch;
	
	
	
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
		
		driver = new Joystick(0);
		
		leftServo = new Servo(8);
		
		rightServo = new Servo(9);
		
		ultra.setAutomaticMode(true); // turns on automatic mode
		
		upperLimitSwitch = new DigitalInput(1);
		
		lowerLimitSwitch = new DigitalInput(2);
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using thef
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
		boolean a = driver.getRawButton(0);
		boolean b = driver.getRawButton(1);
		
		
		if (a) {
			leftServo.setAngle(180);
			rightServo.setAngle(180);
			

		}
		if (b) {
			leftServo.setAngle(0);
			rightServo.setAngle(0);
		}
		if (upperLimitSwitch.get()) {
			if (driver.getRawAxis())
				
		}
	}
	
	public void disabledPeriodic() {
		System.out.println("Distance: " + ultrasonicSample());
	}

	//hi my name is stephen and i am the coolest person in the entire world
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
    public double ultrasonicSample() {
    	double range = ultra.getRangeInches(); // reads the range on the ultrasonic sensor
    	return range;
    }

    public void updateSmartDashboard() {
		
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());
		SmartDashboard.putNumber("Black Ultrasonic", ultra.getRangeInches());
		SmartDashboard.putNumber("Yellow Ultrasonic", ultra.getRangeInches());

		SmartDashboard.putNumber("Left Encoder Count", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Count", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
	}
    
}