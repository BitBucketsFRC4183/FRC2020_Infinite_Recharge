package frc.robot.utils.math;

public class Vector3d {
    private final double x;
    private final double y;
    private final double z;


    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }



    public Vector3d cross(Vector3d b) {
        /*
         *         | x    y    z |
         * a x b = | ax  ay   az | = (ay bz - az by) x - (ax bz - az bx) y + (ax by - ay bx) z
         *         | bx  by   bz |
         */

        double bx = b.getX();
        double by = b.getY();
        double bz = b.getZ();

        return new Vector3d(
            y*bz - z*by,
            -(x*bz - z*bx),
            x*by - y*bx
        );
    }
}