package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BlockArm {
    private ColorSensorClass colorSensorClass;
    private Servo armServo;
    private Servo clampServo;
    HardwareMap hwMap           =  null;

    private final double scanPosition = 0.31;
    private final double armUpPosition = 0.1;
    private final double grabBlockPosition = 0.35;

    private final double clampedBlockPosition = 0.8;
    private final double clampReleasePosition = 0.;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        colorSensorClass = new ColorSensorClass();
        colorSensorClass.init(hwMap);
        armServo = hwMap.get(Servo.class, "armServo");
        clampServo = hwMap.get(Servo.class, "blockClampServo");
        clampServo.setDirection(Servo.Direction.REVERSE);

        clampServo.setPosition(clampedBlockPosition);
        armServo.setPosition(armUpPosition);

    }

    public void activateArm() {
        armServo.setPosition(grabBlockPosition);
    }

    public void scanPosition() {
        armServo.setPosition(scanPosition);
    }

    public void armUp() {
        armServo.setPosition(armUpPosition);
        activateClamp();
    }

    public void activateClamp() {
        clampServo.setPosition(clampedBlockPosition);
    }

    public void deactivateClamp() {
        clampServo.setPosition(clampReleasePosition );
    }

    public String getColor() {
        return colorSensorClass.getColor();
    }

}
