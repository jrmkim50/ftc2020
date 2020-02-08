package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Lift {

    private DcMotor liftMotor;
    private DcMotor liftMotorTwo;
    HardwareMap hwMap           =  null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        this.liftMotor  = hwMap.get(DcMotor.class, "lift");
        this.liftMotorTwo  = hwMap.get(DcMotor.class, "liftTwo");
        this.liftMotor = liftMotor;
        this.liftMotorTwo = liftMotorTwo;
        this.liftMotor.setPower(0);
        this.liftMotorTwo.setPower(0);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        power = Range.clip(power,-0.75,0.75);
        liftMotor.setPower(-power);
        liftMotorTwo.setPower(power);
    }

    public void shutDown() {
        liftMotor.setPower(0);
        liftMotorTwo.setPower(0);
    }

}
