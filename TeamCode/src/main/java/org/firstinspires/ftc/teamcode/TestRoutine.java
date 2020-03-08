package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="TestRoutine", group="Autonomous")

public class TestRoutine extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();
    BlockArm blockArm = new BlockArm();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); //x: 48 30 y: -54 -54. start facing backwards
        intake.init(hardwareMap);
        foundation.init(hardwareMap);
        blockArm.init(hardwareMap);

        waitForStart();

        blockArm.activateClamp();
        sleep(100);

    }
}