package org.usfirst.frc6647.subsystems;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.loops.ILooper;
import org.usfirst.lib6647.loops.Loop;
import org.usfirst.lib6647.loops.LoopType;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.SuperSubsystem;
import org.usfirst.lib6647.subsystem.hypercomponents.HyperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperProfiledPID;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;
import org.usfirst.lib6647.wpilib.LooperRobot;
import org.usfirst.lib6647.wpilib.ProfiledPIDController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Example implementation of a Chassis {@link SuperSubsystem Subsystem}, with
 * angle-arcade control.
 */
public class Chassis extends SuperSubsystem implements SuperAHRS, SuperProfiledPID, SuperTalon, SuperVictor {
	/** {@link DifferentialDrive} used by this {@link SuperSubsystem Subsystem}. */
	private DifferentialDrive drive;
	/** {@link JController} instance used by the Robot. */
	private JController joystick;
	/** {@link HyperAHRS} instance of the Robot's NavX. */
	private HyperAHRS navX;

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
		initProfiledPIDs(robotMap, getName());
		initTalons(robotMap, getName());
		initVictors(robotMap, getName());

		// Additional initialization can be done here.
		getVictor("backLeft").follow(getTalon("frontLeft"));
		getVictor("backRight").follow(getTalon("frontRight"));

		drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
		joystick = Robot.getInstance().getJoystick("driver1");
		navX = getAHRS("navX");

		configureButtonBindings();
	}

	/**
	 * Throw all Command initialization and {@link JController} binding for this
	 * {@link SuperSubsystem} into this method.
	 */
	public void configureButtonBindings() {
		joystick.get("LTrigger").and(joystick.get("RTrigger"))
				.whileActiveContinuous(new InstantCommand(() -> System.out.println("Test!")), true);
	}

	@Override
	public void registerLoops(ILooper looper) {
		looper.register(new Loop() {
			private ProfiledPIDController controller;

			@Override
			public void onFirstStart(double timestamp) {
			}

			@Override
			public void onStart(double timestamp) {
				joystick = Robot.getInstance().getJoystick("driver1");
				controller = getProfiledPIDController("gyro");

				navX.reset();

				synchronized (Chassis.this) {
					controller.reset(navX.getYaw());
					System.out.println("Started angle-arcade drive at: " + timestamp + "!");
				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Chassis.this) {
					var output = controller.calculate(navX.getYaw(),
							Math.abs(joystick.getY()) > 0.15 || Math.abs(joystick.getX()) > 0.15
									? joystick.getAngleDegrees(Hand.kRight)
									: navX.getYaw());
					drive.arcadeDrive(joystick.getY(Hand.kLeft), output, false);
				}
			}

			@Override
			public void onStop(double timestamp) {
				System.out.println("Stopped angle-arcade drive at: " + timestamp + ".");
			}

			@Override
			public LoopType getType() {
				return LoopType.TELEOP;
			}
		});
	}
}