package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BLUE Side forward Autonomous: GLYPH and JEWEL", group="Autonomous")
public class Auton extends LinearOpMode{
    Robot robot = new Robot();
    Wheel rfm;
    Wheel lfm;
    Wheel rbm;
    Wheel lbm;

    public void runOpMode() {
        robot.init(hardwareMap);
        rfm = new Wheel(robot.rightFrontMotor);
        lfm = new Wheel(robot.leftFrontMotor);
        rbm = new Wheel(robot.rightBackMotor);
        lbm = new Wheel(robot.leftBackMotor);
        rfm.runTicks(1024);
        lfm.runTicks(1024);
        rbm.runTicks(1024);
        lbm.runTicks(1024);
    }
}
