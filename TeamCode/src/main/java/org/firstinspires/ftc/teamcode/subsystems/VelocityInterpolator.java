package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.linalg.Matrix;
import dev.nextftc.linalg.N2;
import dev.nextftc.linalg.Vector;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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

    public static double timeToShoot = 0.4;

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

    /**
     * approximation of sin(x)/x using a taylor polynomial for small values of x
     */
    private static double sinc(double x) {
        if (Math.abs(x) < 1e-3) return 1.0 - x * x / 6.0;
        return Math.sin(x) / x;
    }

    /**
     * approximation of (cos(x) - 1)/x using a taylor polynomial for small values of x
     */
    private static double cosc(double x) {
        if (Math.abs(x) < 1e-3) return -x / 2.0 + x * x * x / 24.0;
        return (Math.cos(x) - 1.0) / x;
    }

    public static Pose predictPosition() {
        Vector<N2> p0 = Vector.of(follower().getPose().getX(), follower().getPose().getY());

        double theta0 = follower().getHeading();
        double omega = follower().getAngularVelocity();
        double thetaT = theta0 + omega * timeToShoot;

        Matrix<N2, N2> RTheta0 = Matrix.from(N2.INSTANCE, N2.INSTANCE, new double[][]{
                {Math.cos(thetaT), -Math.sin(thetaT)},
                {Math.sin(thetaT), Math.cos(thetaT)},
        });

        Matrix<N2, N2> Q = Matrix.from(N2.INSTANCE, N2.INSTANCE, new double[][]{
                {timeToShoot * sinc(omega * timeToShoot), timeToShoot * cosc(omega * timeToShoot)},
                {-timeToShoot * cosc(omega * timeToShoot), timeToShoot * sinc(omega * timeToShoot)},
        });

        com.pedropathing.math.Vector velocity = follower().getVelocity();
        velocity.rotateVector(-theta0);
        Vector<N2> v = Vector.of(velocity.getXComponent(), velocity.getYComponent());

        Vector<N2> pT = p0.plus(new Vector<>(RTheta0.times(Q).times(v)));

        return new Pose(pT.get(0), pT.get(1), thetaT);
    }
}
