package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest", group="final")
public class ServoTest extends OpMode {
    private double positionOne = 0.5;
    private double positionTwo = 1;
    private double positionThree = 0.15;

    private CRServo servoOne;
    private CRServo servoTwo;
    private CRServo servoThree;


    @Override
    public void init() {
        servoOne = hardwareMap.get(CRServo.class, "servoOne");
        servoTwo = hardwareMap.get(CRServo.class, "servoTwo");
        servoThree = hardwareMap.get(CRServo.class, "servoThree");

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servoOne.setPower(0.0);
            servoTwo.setPower(0.);
            servoThree.setPower(0.0);
        }

        if (gamepad1.b) {
            servoOne.setPower(-1.0);
            servoTwo.setPower(1.0);
            servoThree.setPower(1);
        }
    }




}
