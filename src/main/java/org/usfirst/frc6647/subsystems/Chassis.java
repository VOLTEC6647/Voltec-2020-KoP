package org.usfirst.frc6647.subsystems;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.subsystem.SuperSubsystem;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;
import org.usfirst.lib6647.wpilib.LooperRobot;

/**
 * Example implementation of a Chassis {@link SuperSubsystem Subsystem},
 * currently does nothing, but it's a good foundation.
 */
public class Chassis extends SuperSubsystem implements SuperAHRS, SuperTalon, SuperVictor {

	/**
	 * A lambda of every {@link SuperSubsystem Subsystem} must be provided to the
	 * {@link Robot}'s {@link LooperRobot super}'s constructor.
	 */
	public Chassis() {
		super("chassis");

		// All SuperComponents must be initialized like this. The 'robotMap' Object is
		// inherited from the SuperSubsystem class, while the second argument is simply
		// this Subsystem's name.
		initAHRS(robotMap, getName());
		initTalons(robotMap, getName());
		initVictors(robotMap, getName());

		// Additional initialization can be done here.
		getVictor("backLeft").follow(getTalon("frontLeft"));
		getVictor("backRight").follow(getTalon("frontRight"));
	}
}