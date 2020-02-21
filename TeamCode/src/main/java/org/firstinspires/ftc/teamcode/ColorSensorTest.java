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
        double sum = colorSensorClassObject.colorSensor.red() +
                     colorSensorClassObject.colorSensor.green() +
                     colorSensorClassObject.colorSensor.blue();
        telemetry.addData("red color sensor: ", colorSensorClassObject.colorSensor.red()/(sum+1E-11));
        telemetry.addData("red color sensor: ", colorSensorClassObject.colorSensor.blue()/(sum+1E-11));
        telemetry.addData("red color sensor: ", colorSensorClassObject.colorSensor.green()/(sum+1E-11));
        // https://aishack.in/tutorials/normalized-rgb/
        telemetry.update();

    }

    @Override
    public void stop() {

    }


}
