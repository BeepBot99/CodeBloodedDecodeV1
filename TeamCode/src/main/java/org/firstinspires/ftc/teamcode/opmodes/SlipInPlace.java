package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Slip in place")
public class SlipInPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");

        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        frontLeft.setPower(1);
        frontRight.setPower(1);
        backLeft.setPower(-1);
        backRight.setPower(-1);

        while (opModeIsActive()) {
        }
    }
}
