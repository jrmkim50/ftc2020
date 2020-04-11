package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleop", group="final")
public class Teleop extends OpMode {
    DriveTrain robot;
    Controller controller1;
    Lift liftMechanism;
    GrabberClass grabber;
    Intake intakeMechanism;
    FoundationMechanism foundation;
    BlockArm blockArm;
    Capstone capstone;

    double rightFrontPower;
    double leftFrontPower;
    double rightBackPower;
    double leftBackPower;
    double liftPower;
    double intakePower;

    @Override
    public void init() {
        telemetry.addLine("PushbotTeleop on");
        robot = new DriveTrain();
        robot.init(hardwareMap);
        controller1 = new Controller(gamepad1);

        liftMechanism = new Lift();
        liftMechanism.init(hardwareMap);

        intakeMechanism = new Intake();
        intakeMechanism.init(hardwareMap);

        grabber = new GrabberClass();
        grabber.init(hardwareMap);

        foundation = new FoundationMechanism();
        foundation.init(hardwareMap);

        blockArm = new BlockArm();
        blockArm.init(hardwareMap);
        blockArm.armUp();

        capstone = new Capstone();
        capstone.init(hardwareMap);
    }

    @Override
    public void loop() {
        rightFrontPower = controller1.getRightFrontPower();
        leftFrontPower = controller1.getLeftFrontPower();
        rightBackPower = controller1.getRightBackPower();
        leftBackPower  = controller1.getLeftBackPower();
        liftPower = gamepad2.left_stick_y;
        intakePower = gamepad2.right_stick_y;

        robot.setPower(rightFrontPower, leftFrontPower, rightBackPower, leftBackPower);

        liftMechanism.setPower(liftPower);
        intakeMechanism.setPower(intakePower);

        if (gamepad2.right_bumper) {
            grabber.clampBlock();
        }
        else if (gamepad2.left_bumper) {
            grabber.releaseBlock();
        }
        if (gamepad2.x) {
            grabber.activePosition();
        }
        else if (gamepad2.y) {
            grabber.neutralPosition();
        }
        if (gamepad2.b) {
            foundation.releaseFoundation();
        }
        else if (gamepad2.a) {
            foundation.clampFoundation();
        }
        if (gamepad2.dpad_down) {
            capstone.dropCapstone();
        }
        else if (gamepad2.dpad_up) {
            capstone.storeCapstone();
        }
    }

    @Override
    public void stop() {
        robot.shutDown();
    }

}