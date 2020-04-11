package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pushbot {

    public DcMotor rightFront, leftFront, rightBack, leftBack;
    private double rightFrontPower, leftFrontPower, rightBackPower, leftBackPower;
    private HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        rightFront = hwMap.get(DcMotor.class, "rfm");
        leftFront = hwMap.get(DcMotor.class, "lfm");
        rightBack = hwMap.get(DcMotor.class, "rbm");
        leftBack = hwMap.get(DcMotor.class, "lbm");
    }

}
