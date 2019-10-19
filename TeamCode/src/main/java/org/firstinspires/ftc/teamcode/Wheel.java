package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wheel {
    private DcMotor wheelMotor;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    public Wheel(DcMotor wheelMotor) {
        this.wheelMotor = wheelMotor;
    }

    public void setPower(double power) {
        wheelMotor.setPower(power);
    }

    public void mecanumEncoderDrive(double speed, double inches, double timeoutS) {
        int newTarget;
        newTarget = wheelMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

        wheelMotor.setTargetPosition(newTarget);
        wheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelMotor.setPower(Math.abs(speed));
        wheelMotor.setPower(0.0);
        wheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runTicks(int ticks) {
        wheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (wheelMotor.getCurrentPosition() < ticks) {
            wheelMotor.setPower(0.5);
        }
        wheelMotor.setPower(0.0);
        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
