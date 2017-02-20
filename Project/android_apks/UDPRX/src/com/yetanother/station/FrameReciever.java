package com.yetanother.station;

public class FrameReciever {
    static {
        System.loadLibrary("fec");
    }

    public native int init();
    public native int destroy();
    public native byte[] readnonblock();
    public native byte[] read();
    public native int SetGimbal(float v);
}
