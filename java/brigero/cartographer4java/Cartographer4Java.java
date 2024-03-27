package brigero.cartographer4java;

public class Cartographer4Java {
	static {
		System.loadLibrary("cartographer4java");
		System.out.println(System.getProperty("java.library.path"));
	}

	private long native_ptr;

	public native void init(String arg1, String arg2, boolean useImu, boolean useOdom, double lidarScanTimeHz);

	public native float posX();

	public native float posY();

	public native void updateLidarData(long timestamp, float[] pointsX, float[] pointsY, float[] intencities);

	public native void stopAndOptimize();

	public native byte[] paintMap();

	public native float angle();
}