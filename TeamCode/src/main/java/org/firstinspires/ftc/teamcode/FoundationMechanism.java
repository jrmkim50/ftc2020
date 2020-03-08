package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationMechanism {
    public Servo clampServoRight;
    public Servo clampServoLeft;
    HardwareMap hwMap = null;

    private double clampServoRightNeutralPosition = 0.0;
    private double clampServoRightActivePosition = 0.5;

    private double clampServoLeftNeutralPosition = 0.;
    private double clampServoLeftActivePosition = 0.5/2;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        clampServoRight = hwMap.get(Servo.class, "clampServoRight");
        clampServoLeft = hwMap.get(Servo.class, "clampServoLeft");
        clampServoRight.setPosition(clampServoRightNeutralPosition);
        clampServoLeft.setDirection(Servo.Direction.REVERSE);
        clampServoLeft.setPosition(clampServoLeftNeutralPosition);
    }

    public void clampFoundation() {
        clampServoRight.setPosition(clampServoRightActivePosition);
        clampServoLeft.setPosition(clampServoLeftActivePosition);
    }

    public void releaseFoundation() {
        clampServoRight.setPosition(clampServoRightNeutralPosition);
        clampServoLeft.setPosition(clampServoLeftNeutralPosition);
    }
}
