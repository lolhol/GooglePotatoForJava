package brigero.cartographer4java;

public class Cartographer3D {
    static {
        System.loadLibrary("cartographer4java3D");
    }

    private long native_ptr;

    public native void init(String dir, String file, double lidarScanTimeHz, String[] imuNames, String[] odomNames, String[] rangeNames);

    public native void addLidarData(long timestamp, String name, float[] pointsX, float[] pointsY, float[] pointsZ, float[] intencities);

    public native void addIMUData(long timestamp, String name, float[] linearAcceleration, float[] angularVelocity);

    public native void addOdomData(long timestamp, String name, float[] position, float[] quaternion);

    public native float[] paintMap();

    public native void stopAndOptimize();

    /**
     * @return this will return 0 = x, 1 = y, 2 = z, 3 = yaw, 4 = pitch, 5 = roll
     */
    public native float[] getPosition();
}