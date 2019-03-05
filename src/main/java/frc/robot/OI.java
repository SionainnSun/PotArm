/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ChangeCargoArmSetpointCommand;
import frc.robot.commands.EnableCargoArmPIDCommand;
import frc.robot.commands.EnableManualCargoArmControlCommandGroup;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick joystick = null;

	public OI(int joystickChannel) {		
		joystick = new Joystick(joystickChannel);	
		JoystickButton a = new JoystickButton(joystick, 1);
		JoystickButton b = new JoystickButton(joystick, 2);
		JoystickButton x = new JoystickButton(joystick, 3);
		JoystickButton y = new JoystickButton(joystick, 4);

		JoystickButton rightJoy = new JoystickButton(joystick, 10);
		rightJoy.whenPressed(new EnableCargoArmPIDCommand());

		JoystickButton leftJoy = new JoystickButton(joystick, 9);
		leftJoy.whenActive(new EnableManualCargoArmControlCommandGroup());

		a.whenPressed(new ChangeCargoArmSetpointCommand(150));
		b.whenPressed(new ChangeCargoArmSetpointCommand(250));
		x.whenPressed(new ChangeCargoArmSetpointCommand(300));
	}

	public Joystick getJoystick() {
		return joystick;
    }
}
