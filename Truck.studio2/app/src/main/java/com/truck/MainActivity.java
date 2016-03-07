package com.truck;

import java.util.Calendar;
import java.util.Formatter;
import java.util.Vector;

import com.truck.R;

import android.os.Bundle;
import android.os.Message;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.CheckBox;
import android.graphics.PorterDuff;

public class MainActivity extends WindowBase implements OnTouchListener {


	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
		Truck.createObjects(this);
 
		SurfaceView canvas = (SurfaceView) findViewById(R.id.canvas);
		SurfaceHolder mSurfaceHolder = canvas.getHolder();
		canvas.setOnTouchListener(this);

		paint = new Paint();
		paint.setDither(true);
		paint.setColor(Color.WHITE);
		paint.setStyle(Paint.Style.STROKE);
		paint.setStrokeJoin(Paint.Join.ROUND);
		paint.setStrokeCap(Paint.Cap.ROUND);
		paint.setStrokeWidth(8);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.activity_main, menu);
        return true;
    }
    
    public boolean onOptionsItemSelected(MenuItem item) 
    {
    	switch (item.getItemId()) 
    	{
        case R.id.menu_settings:
        	startActivity(new Intent(this, SettingsWin.class));
        	return true;
        case R.id.menu_drive:
        	startActivity(new Intent(this, DriveWin.class));
        	return true;

        default:
        	return super.onOptionsItemSelected(item);
    	}
    }

	@Override
	public boolean onTouch(View view, MotionEvent motionEvent) 
	{
		return true;
	}

    public void onClick(View view)
    {
        switch (view.getId())
  		{
        case R.id.drive:
        	startActivity(new Intent(this, DriveWin.class));
        	break;
        case R.id.settings:
        	startActivity(new Intent(this, SettingsWin.class));
        	break;
        case R.id.reset_gyro:
        	Truck.needReset = true;
        	break;
		case R.id.do_mag:
			Truck.needMag = ((CheckBox)view).isChecked();
			if(!Truck.needMag)
			{
				// save the settings when exiting mag calibration
				Truck.needConfig = true;
			}
			break;
  		}
  	}
    
	public void updateGUI()
	{
		SurfaceView canvas1 = (SurfaceView) findViewById(R.id.canvas);
		SurfaceHolder mSurfaceHolder = canvas1.getHolder();
		Canvas canvas = mSurfaceHolder.lockCanvas(null);
		if(canvas != null)
		{
			Settings.initCanvas(canvas);
			
			if(battery == null)
			{
				int x = Settings.border;
				int y = Settings.border;
				Log.v("MainActivity", "updateGUI");
				readouts.add(battery = new LargeReadout(x, y, "BATTERY:", "0.00"));
				battery.x = canvas.getWidth() / 2 - battery.getW(paint) / 2;
				y += battery.getH(paint);
				
				int positions[] = new int[]
				{
						Settings.screenW * 1 / 3,
						Settings.screenW * 2 / 3
				};
				int positions2[] = new int[]
				{
						Settings.screenW * 1 / 4,
						Settings.screenW * 2 / 4,
						Settings.screenW * 3 / 4
				};

				readouts.add(batteryAnalog = new SmallReadout(x, y, 1));
				batteryAnalog.setPositions(positions);
				batteryAnalog.setTitle("Analog:");
				y += batteryAnalog.getH(paint);
				

				SmallReadout gyroTitle = null;
				readouts.add(gyroTitle = new SmallReadout(x, y, 2));
				gyroTitle.setPositions(positions);
				gyroTitle.setTitle("Gyro");
				gyroTitle.update(0, "Center");
				gyroTitle.update(1, "Range");
				y += gyroTitle.getH(paint);
				
				readouts.add(gyro = new SmallReadout(x, y, 2));
				gyro.setPositions(positions);
				gyro.update(0, "0");
				gyro.update(1, "0");
				y += gyro.getH(paint);

				readouts.add(heading = new SmallReadout(x, y, 2));
				heading.setTitle("Heading");
				heading.setPositions(positions);
				heading.update(0, "0.0");
				y += heading.getH(paint);
				
//				readouts.add(power = new SmallReadout(x, y, 2));
//				power.setTitle("Power");
//				power.setPositions(positions);
//				power.update(0, "0.0");
//				y += power.getH(paint);
//				readouts.add(path_x = new SmallReadout(x, y, 2));
//				path_x.setTitle("Path X");
//				path_x.setPositions(positions);
//				path_x.update(0, "0");
//				y += path_x.getH(paint);

				readouts.add(rpm = new SmallReadout(x, y, 2));
				rpm.setTitle("RPM");
				rpm.setPositions(positions);
				rpm.update(0, "0.0");
				y += rpm.getH(paint);

				readouts.add(mag_x = new SmallReadout(x, y, 3));
				mag_x.setTitle("MAG X:");
				mag_x.setPositions(positions2);
				y += mag_x.getH(paint);

				readouts.add(mag_y = new SmallReadout(x, y, 3));
				mag_y.setTitle("MAG Y:");
				mag_y.setPositions(positions2);
				y += mag_y.getH(paint);

				readouts.add(mag_z = new SmallReadout(x, y, 3));
				mag_z.setTitle("MAG Z:");
				mag_z.setPositions(positions2);
				y += mag_z.getH(paint);

				readouts.add(messages = new SmallReadout(x, y, 1));
				messages.update(0, Settings.message);
			}
			
			
			battery.setValue(Truck.truck.battery_voltage);
			batteryAnalog.update(0, Integer.toString(Truck.battery_analog));
			
			gyro.update(0, Integer.toString(Truck.truck.gyro_center));
			gyro.update(1, Integer.toString(Truck.truck.gyro_range));
			heading.update(0, 
				new Formatter(
					new StringBuilder())
						.format("%.02f", Math.fromRad(Truck.current_heading))
						.toString());
//			power.update(0, 
//					new Formatter(
//						new StringBuilder())
//							.format("%.02fW", Truck.power)
//							.toString());
//			path_x.update(0,
//					new Formatter(
//						new StringBuilder())
//							.format("%d", Truck.path_x)
//							.toString());
			rpm.update(0, 
					new Formatter(
						new StringBuilder())
							.format("%d", Truck.rpm)
							.toString());

			mag_x.update(0, new Formatter(new StringBuilder()).format("%d", Truck.mag_x_min).toString());
			mag_x.update(1, new Formatter(new StringBuilder()).format("%d", Truck.mag_x_max).toString());
			mag_y.update(0, new Formatter(new StringBuilder()).format("%d", Truck.mag_y_min).toString());
			mag_y.update(1, new Formatter(new StringBuilder()).format("%d", Truck.mag_y_max).toString());
			mag_z.update(0, new Formatter(new StringBuilder()).format("%d", Truck.mag_z_min).toString());
			mag_z.update(1, new Formatter(new StringBuilder()).format("%d", Truck.mag_z_max).toString());

			
			synchronized(Truck.truck)
			{
			    if(Settings.haveMessage)
			    {
					messages.update(0, Settings.message);
					Settings.haveMessage = false;
			    }
			}
			
			canvas.drawColor(0, PorterDuff.Mode.CLEAR);
			for (int i = 0; i < readouts.size(); i++) {
				((Container) readouts.get(i)).draw(canvas, paint);
			}
			mSurfaceHolder.unlockCanvasAndPost(canvas);
		}
	}

        
    Vector<Container> readouts = new Vector<Container>();
	LargeReadout battery;
	SmallReadout gyro, heading, mag_x, mag_y, mag_z, rpm;
	SmallReadout batteryAnalog;
	SmallReadout messages;
    boolean initialized = false;
    private Paint paint;
 }
