package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {
    public DcMotor  rightFrontMotor = null;
    public DcMotor  leftFrontMotor = null;
    public DcMotor  rightBackMotor = null;
    public DcMotor  leftBackMotor = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        rightFrontMotor  = hwMap.get(DcMotor.class, "rfm");
        leftFrontMotor  = hwMap.get(DcMotor.class, "lfm");
        rightBackMotor = hwMap.get(DcMotor.class, "rbm");
        leftBackMotor  = hwMap.get(DcMotor.class, "lbm");

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shutDown() {
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
    }
}
