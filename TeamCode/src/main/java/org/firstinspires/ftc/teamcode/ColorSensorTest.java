package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensorTest", group="final")
public class ColorSensorTest extends OpMode {
    Controller controller1;
    ColorSensorClass colorSensorClassObject;
    ColorSensor colorSensor;


    @Override
    public void init() {
        telemetry.addLine("Pushbot on");
        colorSensorClassObject = new ColorSensorClass();
        colorSensorClassObject.init(hardwareMap);
        controller1 = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        telemetry.addData("red color sensor: ", colorSensorClassObject.colorSensor.red());

    }

    @Override
    public void stop() {

    }


}
