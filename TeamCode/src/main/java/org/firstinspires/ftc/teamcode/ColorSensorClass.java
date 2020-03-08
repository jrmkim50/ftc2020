package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorSensorClass {

    public ColorSensor colorSensor;

    HardwareMap hwMap           =  null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
    }

    public String getColor() {
        if (((colorSensor.red() * colorSensor.green() * 1.0) / (colorSensor.blue() * colorSensor.blue())) <= 2) {
            return "black";
        } else {
            return "yellow";
        }
    }
}
