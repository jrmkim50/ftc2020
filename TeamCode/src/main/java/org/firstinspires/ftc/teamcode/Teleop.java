package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop", group="final")
public class Teleop extends OpMode{
    DriveTrain robot;
    Controller controller1;
    Wheel rfm;
    Wheel lfm;
    Wheel rbm;
    Wheel lbm;

    double rightFrontPower;
    double leftFrontPower;
    double rightBackPower;
    double leftBackPower;

    @Override
    public void init() {
        telemetry.addLine("Pushbot on");
        robot = new DriveTrain();
        robot.init(hardwareMap);
        controller1 = new Controller(gamepad1);
        rfm = new Wheel(robot.rightFrontMotor);
        lfm = new Wheel(robot.leftFrontMotor);
        rbm = new Wheel(robot.rightBackMotor);
        lbm = new Wheel(robot.leftBackMotor);
    }

    @Override
    public void loop() {
        rightFrontPower = controller1.getRightFrontPower();
        leftFrontPower = controller1.getLeftFrontPower();
        rightBackPower = controller1.getRightBackPower();
        leftBackPower  = controller1.getLeftBackPower();

        lfm.setPower(leftFrontPower);
        rfm.setPower(rightFrontPower);
        lbm.setPower(leftBackPower);
        rbm.setPower(rightBackPower);
    }

    @Override
    public void stop() {
        robot.shutDown();
    }

}