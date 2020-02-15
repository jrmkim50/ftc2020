package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="AutonomousRed", group="Autonomous")

public class AutonRed extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intake.init(hardwareMap);
        foundation.init(hardwareMap);

        waitForStart();
        sleep(500);

    }
}