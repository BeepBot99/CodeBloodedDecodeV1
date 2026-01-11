package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import dev.nextftc.extensions.pedro.PedroComponent;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;


@Config
public final class VelocityInterpolator {
    public static final Interpolation2D farInterpolation = new BilinearInterpolation(
            new double[]{50, 70, 100},
            new double[]{11, 35},
            new double[][]{
                    {1680, 1600},
                    {1600, 1550},
                    {1550, 1500}
            });
    public static final Interpolation2D closeInterpolation = new BilinearInterpolation(
            new double[]{35, 53, 71, 90},
            new double[]{65, 83, 102, 120, 137},
            new double[][]{
                    {1580, 1540, 1480, 1480, 1450},
                    {1490, 1460, 1440, 1420, 1400},
                    {1420, 1400, 1370, 1360, 1340},
                    {1430, 1400, 1390, 1390, 1380}
            }
    );
    public static boolean useRegression = true;

    @SuppressWarnings("SuspiciousNameCombination")
    public static void setVelocityFromLocation() {
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();


        if (useRegression) {
            if (y <= 48) {
                Shooter.onTarget = farInterpolation.interpolate(x, y);
            } else {
                Shooter.onTarget = closeInterpolation.interpolate(x, y);
            }
        }
    }
}
