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
			if(!initialized)
			{
				initialized = true;

				// set up extents
				Settings.border = canvas.getHeight() / 20;
				Settings.margin = canvas.getHeight() / 20;
				int x = Settings.border;
				int y = Settings.border;
				
				Settings.big_font_size = canvas.getHeight() / 6;
				Settings.small_font_size = canvas.getHeight() / 12;
				Settings.screenW = canvas.getWidth();
				
				
				
				readouts.add(battery = new LargeReadout(x, y, "BATTERY:", "0.00"));
				battery.x = canvas.getWidth() / 2 - battery.getW(paint) / 2;
				y += battery.getH(paint) + Settings.margin;
			}
			
			
			
			battery.setValue(Truck.truck.battery_voltage);
			
			
				
			
			canvas.drawColor(0, PorterDuff.Mode.CLEAR);
			for (int i = 0; i < readouts.size(); i++) {
				((Container) readouts.get(i)).draw(canvas, paint);
			}
			mSurfaceHolder.unlockCanvasAndPost(canvas);
		}
	}

    public void handleMessage(Message msg)
    {
		TextView text = (TextView)findViewById(R.id.messages);
		if(text != null) text.setText(msg.getData().getString("text"));
    }
        
    Vector<Container> readouts = new Vector<Container>();
	LargeReadout battery;
    boolean initialized = false;
    private Paint paint;
 }
