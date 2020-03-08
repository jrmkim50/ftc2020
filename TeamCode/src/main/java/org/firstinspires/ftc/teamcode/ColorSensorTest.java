package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensorTest", group="final")
public class ColorSensorTest extends OpMode {
    HardwareMap hwMap           =  null;
    BlockArm blockArm;


    @Override
    public void init() {
        blockArm = new BlockArm();
        blockArm.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("color: ", blockArm.getColor());
        telemetry.update();

        if (gamepad1.dpad_down) {
            blockArm.activateArm();
        } else if (gamepad1.dpad_up) {
            blockArm.scanPosition();
        } else if (gamepad1.dpad_right) {
            blockArm.activateClamp();
        } else if (gamepad1.dpad_left) {
            blockArm.deactivateClamp();
        }

    }

    @Override
    public void stop() {

    }


}
