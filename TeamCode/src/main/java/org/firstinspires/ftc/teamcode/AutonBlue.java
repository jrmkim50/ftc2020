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

@Autonomous(name="AutonomousBlue", group="Autonomous")

public class AutonBlue extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();

    BNO055IMU imu;
    Orientation angles;

    final double RADIUS = 2;
    final double TICKS_PER_REVOLUTION = 537.6;
    final double COUNTS_PER_INCH = TICKS_PER_REVOLUTION/(2 * Math.PI * RADIUS);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intake.init(hardwareMap);
        foundation.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();
        sleep(500);

        robot.mecanumDriveStraightAlongZero(0.5, 20, imu, COUNTS_PER_INCH);
        robot.mecanumStrafe(0.5, 20, imu, MovementDirection.RIGHT, COUNTS_PER_INCH);
        robot.mecanumStrafe(0.5, 20, imu, MovementDirection.LEFT, COUNTS_PER_INCH);
        robot.mecanumDriveStraightAlongZero(0.5, -20, imu, COUNTS_PER_INCH);
        robot.turnRobot(0.5, 90, imu);
        robot.mecanumDriveStraightAlongZero(0.5, 30, imu, COUNTS_PER_INCH);
        robot.mecanumStrafe(0.5, 30, imu, MovementDirection.RIGHT, COUNTS_PER_INCH);
        robot.mecanumStrafe(0.5, 30, imu, MovementDirection.LEFT, COUNTS_PER_INCH);
        robot.mecanumDriveStraightAlongZero(0.5, -30, imu, COUNTS_PER_INCH);
        robot.turnRobot(0.5, 180, imu);
    }
}