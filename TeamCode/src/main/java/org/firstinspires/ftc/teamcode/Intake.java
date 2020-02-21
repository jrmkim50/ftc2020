package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Intake {

    public DcMotor intakeMotorOne;
    public DcMotor intakeMotorTwo;
    HardwareMap hwMap           =  null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        this.intakeMotorOne  = hwMap.get(DcMotor.class, "intakeOne");
        this.intakeMotorOne.setPower(0);
        this.intakeMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.intakeMotorTwo  = hwMap.get(DcMotor.class, "intakeTwo");
        this.intakeMotorTwo.setDirection(DcMotor.Direction.REVERSE);
        this.intakeMotorTwo.setPower(0);
        this.intakeMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        power = Range.clip(power,-0.75,0.75);
        intakeMotorOne.setPower(power);
        intakeMotorTwo.setPower(power);
    }

    public void shutDown() {
        intakeMotorOne.setPower(0);
        intakeMotorTwo.setPower(0);
    }

}
