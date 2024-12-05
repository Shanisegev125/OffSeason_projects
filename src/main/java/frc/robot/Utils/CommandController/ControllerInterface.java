package frc.robot.Utils.CommandController;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerInterface {

    default Trigger up() {return null;}
    default Trigger down() {return null;}
    default Trigger left() {return null;}
    default Trigger right() {return null;}

    default Trigger pov0() {return null;}
    default Trigger pov180() {return null;}
    default Trigger pov90() {return null;}
    default Trigger pov270() {return null;}

    default Trigger rightStick() {return null;}
    default Trigger leftStick() {return null;}

    default Trigger rightTrigger() {return null;}
    default Trigger leftTrigger() {return null;}

    default Trigger leftBumper() {return null;}
    default Trigger rightBumper() {return null;}

    default double getRightX() {return 0;}
    default double getRightY() {return 0;}
    default double getLeftX() {return 0;}
    default double getLeftY() {return 0;}

    default double getRightTrigger() {return 0;}
    default double getLeftTrigger() {return 0;}

    default double getRawAxis(int axis) {return 0;}
}
