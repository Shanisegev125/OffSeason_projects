package frc.robot.Utils.CommandController;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5Controller implements ControllerInterface {

    CommandPS5Controller controller;

    public PS5Controller(int port) {
        this.controller = new CommandPS5Controller(port);
    }

    public Trigger up() {
        return controller.triangle();
    }

    public Trigger down() {
        return controller.cross();
    }

    public Trigger left() {
        return controller.square();
    }

    public Trigger right() {
        return controller.circle();
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
        return controller.R3();
    }

    public Trigger leftStick() {
        return controller.L3();
    }

    public Trigger rightTrigger() {
        return controller.R2();
    }

    public Trigger leftTrigger() {
        return controller.L2();
    }

    public Trigger leftBumper() {
        return controller.L1();
    }

    public Trigger rightBumper() {
        return controller.R1();
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
        return controller.getR2Axis();
    }

    public double getLeftTrigger() {
        return controller.getL2Axis();
    }

    public double getRawAxis(int axis) {
        return controller.getRawAxis(axis);
    }
}
