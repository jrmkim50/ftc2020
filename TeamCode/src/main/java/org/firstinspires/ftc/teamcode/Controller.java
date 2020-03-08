package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Controller {
    private Gamepad gamepad;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public double getRightFrontPower() {
        double rightFrontPower = 0;
        rightFrontPower = -gamepad.left_stick_y - gamepad.right_stick_x - gamepad.left_stick_x;

        rightFrontPower = Range.clip(rightFrontPower,-0.9,0.9);

        if (gamepad.right_bumper) {
            rightFrontPower *= 0.4;
        }
        return rightFrontPower;
    }

    public double getLeftFrontPower() {
        double leftFrontPower = 0;
        leftFrontPower = -gamepad.left_stick_y + gamepad.right_stick_x + gamepad.left_stick_x;

        leftFrontPower = Range.clip(leftFrontPower,-0.9,0.9);

        if (gamepad.right_bumper) {
            leftFrontPower *= 0.4;
        }
        return leftFrontPower;
    }

    public double getRightBackPower() {
        double rightBackPower = 0;
        rightBackPower = -gamepad.left_stick_y - gamepad.right_stick_x + gamepad.left_stick_x;

        rightBackPower = Range.clip(rightBackPower,-0.9,0.9);

        if (gamepad.right_bumper) {
            rightBackPower *= 0.4;
        }
        return rightBackPower;
    }

    public double getLeftBackPower() {
        double leftBackPower = 0;
        leftBackPower  = -gamepad.left_stick_y + gamepad.right_stick_x - gamepad.left_stick_x;

        leftBackPower = Range.clip(leftBackPower,-0.9,0.9);

        if (gamepad.right_bumper) {
            leftBackPower *= 0.4;
        }
        return leftBackPower;
    }
}
