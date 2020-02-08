package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GrabberClass {

    public Servo turnServo;
    public Servo clampServo;
    public Servo linearServo;
    private double turnServoForwardPosition = 0.85;
    private double turnServoBackwardPosition = 0.15;

    private double clampServoClosePosition = 0.4;
    private double clampServoOpenPosition = 0.9;

    private double linearServoForwardPosition = 0.0;
    private double linearServoBackwardPosition = 0.9;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        turnServo= hwMap.get(Servo.class, "turnServo");
        clampServo= hwMap.get(Servo.class, "clampServo");
        linearServo= hwMap.get(Servo.class, "linearServo");
        turnServo.setPosition(turnServoForwardPosition);
        clampServo.setPosition(clampServoOpenPosition);
        linearServo.setPosition(linearServoForwardPosition);
    }

    public void clampBlock() {
        clampServo.setPosition(clampServoClosePosition);
    }

    public void releaseBlock() {
        clampServo.setPosition(clampServoOpenPosition);
    }

    public void activePosition() {
        linearServo.setPosition(linearServoBackwardPosition);
        turnServo.setPosition(turnServoBackwardPosition);
    }

    public void neutralPosition() {
        turnServo.setPosition(turnServoForwardPosition);
        linearServo.setPosition(linearServoForwardPosition);
    }

}
