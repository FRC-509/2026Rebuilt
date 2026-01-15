package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Handle input from Thrustmaster T.16000M joysticks connected to the Driver Station.
*/
public class ThrustmasterJoystick extends Joystick {
	public enum StickButton {
		Trigger(1),
		Bottom(2),
		Left(3),
		Right(4),
		LeftSideLeftTop(5),
		LeftSideMiddleTop(6),
		LeftSideRightTop(7),
		LeftSideRightBottom(8),
		LeftSideMiddleBottom(9),
		LeftSideLeftBottom(10),
		RightSideRightTop(11),
		RightSideMiddleTop(12),
		RightSideLeftTop(13),
		RightSideLeftBottom(14),
		RightSideMiddleBottom(15),
		RightSideRightBottom(16);

		public final int id;

		StickButton(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	/**
	 * Constructs an instance of a Thrustmaster T.16000M joystick.
	 *
	 * @param port The port index on the Driver Station that the device is plugged into.
	 */
	public ThrustmasterJoystick(int port) {
		super(port);
	}

	/**
	 * Returns a joystick button for triggering commands.
	 *
	 * @param button The physical button on the controller.
	 * @return A JoystickButton object attached to the given controller button.
	 */
	public JoystickButton getJoystickButton(StickButton button) {
		return new JoystickButton(this, button.id());
	}

	/**
	 * Whether the button was pressed since the last check.
	 * <p>This method returns true if the button went from not pressed to held down since the last time this method was called. This is useful if you only want to call a function once when you press the button.
	 *
	 * @param button The physical button on the controller.
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean isPressed(StickButton button) {
		return super.getRawButtonPressed(button.id());
	}

	/**
	 * Binds a command to when a button is pressed.
	 * <p>The command is ran if the button went from not pressed to held down. This is useful if you only want to execute a command once when you press the button.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute once the button is pressed.
	 */
	public void isPressedBind(StickButton button, Command command) {
		getJoystickButton(button).toggleOnTrue(command);
	}

	/**
	 * Whether the button was released since the last check.
	 * <p>This method returns true if the button went from held down to not pressed since the last time this method was called. This is useful if you only want to call a function once when you release the button.
	 * 
	 * @param button The physical button on the controller.
	 * @return Whether the button was released since the last check.
	 */
	public boolean isReleased(StickButton button) {
		return super.getRawButtonReleased(button.id());
	}

	/**
	 * Binds a command to when a button is released.
	 * <p>The command is ran if the button went from held down to not pressed. This is useful if you only want to execute a command once when you release the button.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute once the button is released.
	 */
	public void isReleasedBind(StickButton button, Command command) {
		getJoystickButton(button).toggleOnFalse(command);
	}

	/**
	 * Get the button value.
	 * <p>This method returns true if the button is being held down at the time that this method is being called.
	 *
	 * @param button The physical button on the controller.
	 * @return The state of the button.
	 */
	public boolean isDown(StickButton button) {
		return super.getRawButton(button.id());
	}

	/**
	 * Binds a command to while a button is held down. The given command is started when the button is initially pressed and is cancelled when the button is released.
	 * <p>Doesn't re-start the command if it ends while the button is still pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute while the button is pressed.
	 */	
	public void isDownBind(StickButton button, Command command) {
		getJoystickButton(button).whileTrue(command);
	}

	/**
	 * This method returns false if the button is being held down at the time that this method is being called.
	 *
	 * @param button The physical button on the controller.
	 * @return Whether the button is up or not.
	 */
	public boolean isUp(StickButton button) {
		return !isDown(button);
	}

	/**
	 * Binds a command to while a button is not pressed. The given command is started when the button is initially released and is cancelled when the button is pressed.
	 * <p>Doesn't re-start the command if it ends while the button still has not been pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute while the button is not pressed.
	 */	
	public void isUpBind(StickButton button, Command command) {
		getJoystickButton(button).whileFalse(command);
	}
}