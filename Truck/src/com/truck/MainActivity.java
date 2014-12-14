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
import android.widget.TextView;
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
				
				int positions[] = new int[] { Settings.screenW * 1 / 3, Settings.screenW * 2 / 3 };
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

			
			synchronized(Truck.truck)
			{
			    if(Settings.haveMessage)
			    {
					TextView text = (TextView)findViewById(R.id.messages);
					if(text != null) text.setText(Settings.message);
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
	SmallReadout gyro, heading;
	SmallReadout batteryAnalog;
    boolean initialized = false;
    private Paint paint;
 }
