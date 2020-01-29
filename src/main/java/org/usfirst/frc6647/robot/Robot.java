package org.usfirst.frc6647.robot;

import org.usfirst.frc6647.subsystems.Chassis;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.SuperSubsystem;
import org.usfirst.lib6647.wpilib.LooperRobot;

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

		joysticks.put("driver1", new JController(0));

		registerSubsystems(Chassis::new);
	}
}
