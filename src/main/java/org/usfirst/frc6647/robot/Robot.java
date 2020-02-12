package org.usfirst.frc6647.robot;

import org.usfirst.frc6647.subsystems.Chassis;
import org.usfirst.lib6647.loops.LooperRobot;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.SuperSubsystem;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Basic implementation of a {@link LooperRobot} with a single instance, an
 * example {@link SuperSubsystem Subsystem} ({@link Chassis}), and a single
 * {@link JController}.
 */
public class Robot extends LooperRobot {
	/** Static {@link Robot} instance. */
	private static Robot instance = null;

	/**
	 * Method for getting currently running {@link Robot} instance.
	 * 
	 * @return instance
	 */
	public synchronized static Robot getInstance() {
		return instance;
	}

	/**
	 * Constructor for this implementation of {@link LooperRobot}, should only need
	 * to be created once, by the {@link Main} class.
	 */
	protected Robot() {
		super();

		if (instance == null) // Might not be necessary, but just in case.
			instance = this;

		initJoysticks();
		registerSubsystems(Chassis::new);
	}

	/**
	 * Run any {@link JController} initialization here.
	 */
	private void initJoysticks() {
		var driver1 = new JController(0);

		System.out.printf("'%s'\n", driver1.getName());

		if (driver1.getName().equals("Wireless Controller")) {
			driver1.setXY(Hand.kLeft, 0, 1);
			driver1.setXY(Hand.kRight, 2, 5);
		} else if (driver1.getName().equals("Generic   USB  Joystick")) {
			driver1.setXY(Hand.kLeft, 0, 1);
			driver1.setXY(Hand.kRight, 2, 4);
		}

		joysticks.put("driver1", driver1);
	}
}
