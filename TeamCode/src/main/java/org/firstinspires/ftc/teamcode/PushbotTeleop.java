package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Teleop", group="final")

public class PushbotTeleop extends OpMode {

    private Pushbot pushbot;
    private double rightFrontPower, leftFrontPower, rightBackPower, leftBackPower;


    @Override
    public void init() {
        pushbot = new Pushbot();
        pushbot.init(hardwareMap);
    }

    @Override
    public void loop() {
        rightFrontPower = gamepad1.right_stick_y;
        leftFrontPower = gamepad1.left_stick_y;
        rightBackPower = gamepad1.right_stick_y;
        leftBackPower = gamepad1.left_stick_y;
        pushbot.rightFront.setPower(rightFrontPower);
        pushbot.leftFront.setPower(leftFrontPower);
        pushbot.rightBack.setPower(rightBackPower);
        pushbot.leftBack.setPower(leftBackPower);


    }
}
