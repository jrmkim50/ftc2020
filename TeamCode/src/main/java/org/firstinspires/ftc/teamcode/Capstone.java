package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Capstone {

    private Servo capstoneDropper;
    double capstoneStorePosition = 0;
    double capstoneDropPosition = .71;
    HardwareMap hwMap           =  null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        this.capstoneDropper= hwMap.get(Servo.class, "capstoneDropper");
        storeCapstone();
    }

    public void storeCapstone() {
        capstoneDropper.setPosition(capstoneStorePosition);
    }

    public void dropCapstone() {
        capstoneDropper.setPosition(capstoneDropPosition);
    }

}
