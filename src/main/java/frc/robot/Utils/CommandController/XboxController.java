package frc.robot.Utils.CommandController;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController implements ControllerInterface {

    private final CommandXboxController controller;

    public XboxController(int port) {
        this.controller = new CommandXboxController(port);
    }

    public Trigger up() {
        return controller.y();
    }

    public Trigger down() {
        return controller.a();
    }

    public Trigger left() {
        return controller.x();
    }

    public Trigger right() {
        return controller.b();
    }

    public Trigger pov0() {
        return controller.povUp();
    }

    public Trigger pov180() {
        return controller.povDown();
    }

    public Trigger pov90() {
        return controller.povRight();
    }

    public Trigger pov270() {
        return controller.povLeft();
    }

    public Trigger rightStick() {
        return controller.rightStick();
    }

    public Trigger leftStick() {
        return controller.leftStick();
    }

    public Trigger rightTrigger() {
        return controller.rightTrigger();
    }

    public Trigger leftTrigger() {
        return controller.leftTrigger();
    }

    public Trigger leftBumper() {
        return controller.leftBumper();
    }

    public Trigger rightBumper() {
        return controller.rightBumper();
    }

    public double getRightX() {
        return controller.getRightX();
    }

    public double getRightY() {
        return controller.getRightY();
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightTrigger() {
        return controller.getRightTriggerAxis();
    }

    public double getLeftTrigger() {
        return controller.getLeftTriggerAxis();
    }

    public double getRawAxis(int axis) {
        return controller.getRawAxis(axis);
    }
}
