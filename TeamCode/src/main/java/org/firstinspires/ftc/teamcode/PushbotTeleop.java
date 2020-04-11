package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Teleop", group="final")

public class PushbotTeleop extends OpMode {

    private Pushbot pushbot;
    private double rightFrontPower, leftFrontPower, rightBackPower, leftBackPower;
    private Controller gamepad1Controller = new Controller(gamepad1);


    @Override
    public void init() {
        pushbot = new Pushbot();
        pushbot.init(hardwareMap);
    }

    @Override
    public void loop() {
        rightFrontPower = gamepad1Controller.getRightFrontPower();
        leftFrontPower = gamepad1Controller.getLeftFrontPower();
        rightBackPower = gamepad1Controller.getRightBackPower();
        leftBackPower = gamepad1Controller.getLeftBackPower();
        pushbot.rightFront.setPower(rightFrontPower);
        pushbot.leftFront.setPower(leftFrontPower);
        pushbot.rightBack.setPower(rightBackPower);
        pushbot.leftBack.setPower(leftBackPower);
    }
}