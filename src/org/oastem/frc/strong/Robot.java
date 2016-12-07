package org.oastem.frc.strong;

import org.oastem.frc.*;
import org.oastem.frc.control.*;
import org.oastem.frc.sensor.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;

import org.oastem.frc.sensor.FRCGyroAccelerometer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled cnby the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions . If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	// Ports
	private final int FRONT_LEFT_CAN_DRIVE = 0;
	private final int FRONT_RIGHT_CAN_DRIVE = 2;
	private final int BACK_LEFT_CAN_DRIVE = 1;
	private final int BACK_RIGHT_CAN_DRIVE = 3;


	// Values
	private final int DRIVE_ENC_CODE_PER_REV = 2048;
	private final int DRIVE_WHEEL_DIAM = 8;

	// Objects
	private TalonDriveSystem talonDrive = TalonDriveSystem.getInstance();
	private LogitechGamingPad pad;
	private LogitechGamingPad padSupport;

	private SmartDashboard dash;

	private CameraServer camera;

	// Joystick commands

	private double slowTrigger;
	private boolean eStop1Pressed;
	private boolean eStop2Pressed;

	public Robot() {
		talonDrive.initializeTalonDrive(FRONT_LEFT_CAN_DRIVE, BACK_LEFT_CAN_DRIVE, FRONT_RIGHT_CAN_DRIVE,
				BACK_RIGHT_CAN_DRIVE, DRIVE_ENC_CODE_PER_REV, DRIVE_WHEEL_DIAM);
	}

	public void robotInit() {
		dash = new SmartDashboard();
		talonDrive.calibrateGyro();
		
		pad = new LogitechGamingPad(0);
		padSupport = new LogitechGamingPad(1);

		camera = CameraServer.getInstance();
		camera.startAutomaticCapture();
	}


	public void autonomous() {
		dash.putString("mode: ", "auto");
	}

	public void operatorControl() {
		boolean stop = false;
		while (isOperatorControl() && isEnabled()) {
			slowTrigger = pad.getLeftTriggerValue();
	
			eStop1Pressed = pad.getBackButton() || padSupport.getBackButton();
			eStop2Pressed = pad.getStartButton() || padSupport.getStartButton();

			if (eStop1Pressed && eStop2Pressed)
				stop = true;

			if (!stop) 
				talonDrive.tankDrive(pad.getLeftAnalogY() * -scaleTrigger(slowTrigger),
						pad.getRightAnalogY() * -scaleTrigger(slowTrigger));
			}
	}
	
	private double scaleTrigger(double trigger) {
		return Math.min(1.0, 1.0 - 0.9 * trigger);
	}


	public void test() {
		while (isTest() && isEnabled()){
			dash.putString("mode: ", "test");
		}
	}
}
