package com.yetanother.station;

import java.nio.ByteBuffer;
import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.media.MediaCodec;
import android.media.MediaCodec.BufferInfo;
import android.media.MediaFormat;
import android.media.MediaPlayer;
import android.media.MediaPlayer.OnPreparedListener;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuItem.OnMenuItemClickListener;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.WindowManager;

public class MainActivity extends Activity implements android.hardware.SensorEventListener, OnPreparedListener
{
	private android.hardware.SensorManager mSensorManager;
	private Sensor mSensor;
	SurfaceView surfaceView;
	SurfaceHolder holder;
	boolean running = false;
	DecodingThread decoder;
	
	@Override
	public void onPause()
	{
		super.onPause();
		mSensorManager.unregisterListener(this);
		try
		{
			running = false;
			decoder.join();
			
			getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
			
		} catch (Exception e)
		{
		}
	}
	
	
	FrameReciever s = new FrameReciever();;
	@Override
	public void onResume()
	{
		super.onResume();
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);
				
		running = true;
	}
	
	
	private class DecodingThread extends Thread {
	
		private MediaCodec decoder = null;
		boolean idr_found = true;
		
		public DecodingThread(Surface surface, int port) {
			decoder = MediaCodec.createDecoderByType("video/avc");
			MediaFormat format = MediaFormat.createVideoFormat("video/avc", 1920, 1080);
			decoder.configure(format, surface, null, 0);
			decoder.start();
		}
		
		public void run()
		{
			s.init();
			while(running)
			{
				byte[] frame = s.readnonblock();
				
				if (frame.length > 0)
				{
					feedDecoder(frame, frame.length);
					Log.e("decoder", "feed " + frame.length +" bytes");
				}
				else
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
			}
			s.destroy();
		}
		
		private void feedDecoder(byte[] n, int len) {
            //Log.i(TAG, "feedDecoder " + n[0] + n[1] + n[2] + n[3] + "  len =" + len);
            ByteBuffer[] inputBuffers = decoder.getInputBuffers();
            decoder.getOutputBuffers();

            if (!idr_found) {
                byte[] sps_pps = new byte[]{0x00, 0x00, 0x00, 0x01, 0x27, 0x64, (byte)0xE0, 0x28, (byte)0xAC, 0x1A, (byte)0xD0, (byte)0x0f, 0x00, 0x44, (byte)0xFC, (byte)0xB0,
                		(byte)0x80, 0x00, 0x03, 0x03, 0x00, (byte)0x80, 0x00, 0x00, (byte)0x0F, (byte)0x07, (byte)0x88, (byte)0x3D, 0x40, 0x00, 0x00, 0x00,
                                              0x01, (byte)0x28, (byte)0xCE, 0x32, 0x48};

                idr_found = true;
                feedDecoder(sps_pps, sps_pps.length);
            }

            int inputBufferIndex = decoder.dequeueInputBuffer(0);
            if (inputBufferIndex >= 0) {
                // fill inputBuffers[inputBufferIndex] with valid data
                //Log.i(TAG, "in index:" + inputBufferIndex);
                inputBuffers[inputBufferIndex].clear();
                inputBuffers[inputBufferIndex].put(n);
                decoder.queueInputBuffer(inputBufferIndex, 0, len, 0, 0);
            }

            BufferInfo bi = new MediaCodec.BufferInfo();
            int outputBufferIndex = decoder.dequeueOutputBuffer(bi, 0);
            //Log.i(TAG, "out index1 :" + outputBufferIndex);


            while (outputBufferIndex >= 0) {
                // outputBuffer is ready to be processed or rendered.
                //Log.i(TAG, "out index:" + outputBufferIndex);
                decoder.releaseOutputBuffer(outputBufferIndex, true);
                outputBufferIndex = decoder.dequeueOutputBuffer(bi, 0);
            }
            if (outputBufferIndex == MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED) {
                decoder.getOutputBuffers();
            } else if (outputBufferIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                decoder.getOutputFormat();
            }
        }
	}
	
	private class surface_cb implements SurfaceHolder.Callback{
		@Override
		public void surfaceChanged(SurfaceHolder holder, int format, int width,	int height) {
		}
		@Override
		public void surfaceCreated(SurfaceHolder holder) {
			
			new DecodingThread(holder.getSurface(), 0xbbb).start();
		}
		@Override
		public void surfaceDestroyed(SurfaceHolder holder) {
		}
	}
	surface_cb cb = new surface_cb();
	

	@Override
	protected void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
		setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);//Ç¿ÖÆÎªºáÆÁ
		
		
		
		surfaceView = (SurfaceView)this.findViewById(R.id.surfaceView1);
		holder = surfaceView.getHolder();
		holder.addCallback(cb);
		holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
		
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu)
	{
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);

		menu.findItem(R.id.connect).setOnMenuItemClickListener(new OnMenuItemClickListener()
		{
			public boolean onMenuItemClick(MenuItem item)
			{
				return true;
			}
		});
		return true;
	}
	private float[] rMatrix = new float[9];

	public void calculateAngles(float[] result, float[] rVector) {
		// caculate rotation matrix from rotation vector first
		SensorManager.getRotationMatrixFromVector(rMatrix, rVector);

		// calculate Euler angles now
		SensorManager.getOrientation(rMatrix, result);

		for (int i = 0; i < result.length; i++) {
			result[i] = (float) Math.toDegrees(result[i]);
		}
	}
	
	float constrain(float in, float low, float high){
		if (in > high)
			return high;
		if (in < low)
			return low;
		return in;
	}

	private float[] euler = new float[9];

	@Override
	public void onSensorChanged(SensorEvent event) {
		calculateAngles(euler, event.values);
		
		euler[1] = constrain(euler[1], -30.0f, +30.0f);
		euler[2] = constrain(euler[2], -65.0f, +65.0f);
		
		//Log.e("euler", "euler[2]="+euler[2]);
		
		s.SetGimbal((float)(euler[2] * Math.PI / 180.0));
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		
	}

	@Override
	public void onPrepared(MediaPlayer mp) {
		Log.e("", "prepared");
		mp.start();
	}
}
