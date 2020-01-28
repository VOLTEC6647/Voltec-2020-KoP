package org.usfirst.frc6647.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc6647.robot.Robot;
import org.usfirst.lib6647.loops.ILooper;
import org.usfirst.lib6647.loops.Loop;
import org.usfirst.lib6647.loops.LoopType;
import org.usfirst.lib6647.oi.JController;
import org.usfirst.lib6647.subsystem.PIDSuperSubsystem;
import org.usfirst.lib6647.subsystem.SuperSubsystem;
import org.usfirst.lib6647.subsystem.hypercomponents.HyperPIDController;
import org.usfirst.lib6647.subsystem.supercomponents.SuperAHRS;
import org.usfirst.lib6647.subsystem.supercomponents.SuperTalon;
import org.usfirst.lib6647.subsystem.supercomponents.SuperVictor;
import org.usfirst.lib6647.wpilib.LooperRobot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Example implementation of a Chassis {@link SuperSubsystem Subsystem}, with
 * angle-arcade control.
 */
public class Chassis extends PIDSuperSubsystem implements SuperAHRS, SuperTalon, SuperVictor {
	/** {@link DifferentialDrive} used by this {@link SuperSubsystem Subsystem}. */
	private DifferentialDrive drive;

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

		drive = new DifferentialDrive(getTalon("frontLeft"), getTalon("frontRight"));
	}

	@Override
	public void registerLoops(ILooper looper) {
		looper.register(new Loop() {
			private JController joystick;
			private HyperPIDController pidGyro;
			private AHRS navX;

			@Override
			public void onFirstStart(double timestamp) {
				getAHRS("navX").reset();
			}

			@Override
			public synchronized void onStart(double timestamp) {
				joystick = Robot.getInstance().getJoystick("driver1");
				pidGyro = getPIDController("gyro");
				navX = getAHRS("navX");

				setSetpoint("gyro", navX.getYaw());
				System.out.println("Started angle-arcade drive at: " + timestamp + "!");
			}

			@Override
			public synchronized void onLoop(double timestamp) {
				setSetpoint("gyro",
						Math.abs(joystick.getRawAxis(4)) > 0.15 || Math.abs(joystick.getRawAxis(2)) > 0.15
								? Math.toDegrees(Math.atan2(joystick.getRawAxis(4), joystick.getRawAxis(2)))
								: navX.getYaw());
				double output = pidGyro.calculate(navX.getYaw());
				drive.arcadeDrive(joystick.getRawAxis(1), output, false);
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