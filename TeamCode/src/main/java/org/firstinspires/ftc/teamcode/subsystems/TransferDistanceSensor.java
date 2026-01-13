package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TransferDistanceSensor implements Subsystem {
    public static final TransferDistanceSensor INSTANCE = new TransferDistanceSensor();
    private Rev2mDistanceSensor distanceSensor;

    public static double distanceThreshold = 80;

    private boolean distanceLocked = false;

    private TransferDistanceSensor() {
    }

    @Override
    public void initialize() {
        distanceSensor = ActiveOpMode.hardwareMap().get(Rev2mDistanceSensor.class, "transfer_distance");
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public static boolean hasBall() {
        return INSTANCE.distanceLocked;
    }

    public static void resetLocked() {
        INSTANCE.distanceLocked = false;
    }

    @Override
    public void periodic() {
        if (getDistance() <= distanceThreshold) distanceLocked = true;
        FtcDashboard.getInstance().getTelemetry().addData("Transfer Distance", getDistance());
        FtcDashboard.getInstance().getTelemetry().addData("Transfer Distance Locked", distanceLocked);
    }
}
