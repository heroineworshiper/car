package com.truck;

import java.util.Vector;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.os.Message;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.TextView;

// fully manual controller

public class DriveWin extends WindowBase implements OnTouchListener
{

	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.drive);
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

	
	public void onPause()
	{
		super.onPause();
		synchronized(Truck.truck)
		{
			Truck.haveControls = false;
		}
		
	}


	public void onStop()
	{
		super.onStop();
		synchronized(Truck.truck)
		{
			Truck.haveControls = false;
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
			if(throttle == null)
			{
				int x = Settings.border;
				int y = Settings.border;
				Settings.stickW = (canvas.getWidth() - 
						Settings.margin * 2) / 2;
				int throttleW = Settings.stickW;

				readouts.add(throttle = new StickPanel(
						0, 
						0, 
						Settings.stickW,
						canvas.getHeight()));

				readouts.add(cyclic = new StickPanel(
						canvas.getWidth() - Settings.stickW, 
						0, 
						Settings.stickW,
						canvas.getHeight()));

			}
			
			synchronized(Truck.truck)
			{
				Truck.throttleOut = (int)(Math.clamp(throttle.userY * 127, -127, 127));
				Truck.steeringOut = (int)(Math.clamp(cyclic.userX * 127, -127, 127));
				Truck.haveControls = throttle.isActive ||
						cyclic.isActive;
				

			    if(Settings.haveMessage)
			    {
					TextView text = (TextView)findViewById(R.id.messages2);
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
	
	@Override
	public boolean onTouch(View view, MotionEvent motionEvent) 
	{
		boolean gotIt = false;
		int pointers = motionEvent.getPointerCount();
		int actionPointerId = (motionEvent.getAction() & MotionEvent.ACTION_POINTER_ID_MASK) >>
			MotionEvent.ACTION_POINTER_ID_SHIFT;
		int actionEnum = motionEvent.getAction() & MotionEvent.ACTION_MASK;

		
		switch(motionEvent.getAction())
		{
// single point
		case MotionEvent.ACTION_DOWN:
			if(!throttleTouch.active &&
					throttle.handlePress(
						(int)motionEvent.getX(), 
						(int)motionEvent.getY()))
				{
					throttleTouch.active = true;
					throttleTouch.id = motionEvent.getPointerId(0);
				}
				else
				if(!cyclicTouch.active &&
					cyclic.handlePress(
						(int)motionEvent.getX(), 
						(int)motionEvent.getY()))
				{
					cyclicTouch.active = true;
					cyclicTouch.id = motionEvent.getPointerId(0);
				}
				break;

		case MotionEvent.ACTION_UP:
			if(throttleTouch.active) throttle.handleRelease();
			if(cyclicTouch.active) cyclic.handleRelease();
			throttleTouch.active = false;
			cyclicTouch.active = false;
			break;

// multi point
		default:
			for(int i = 0; i < pointers; i++)
			{
				if(motionEvent.getPointerId(i) == actionPointerId)
				{
					switch(actionEnum)
					{
					case MotionEvent.ACTION_POINTER_DOWN:
						if(!throttleTouch.active &&
							throttle.handlePress(
								(int)motionEvent.getX(i), 
								(int)motionEvent.getY(i)))
						{
							throttleTouch.active = true;
							throttleTouch.id = actionPointerId;
						}
						else
						if(!cyclicTouch.active &&
							cyclic.handlePress(
								(int)motionEvent.getX(i), 
								(int)motionEvent.getY(i)))
						{
							cyclicTouch.active = true;
							cyclicTouch.id = actionPointerId;
						}
						break;
						
					case MotionEvent.ACTION_POINTER_UP:
						if(throttleTouch.active && 
							throttleTouch.id == actionPointerId)
						{
							throttle.handleRelease();
							throttleTouch.active = false;
						}
						else
						if(cyclicTouch.active &&
							cyclicTouch.id == actionPointerId)
						{
							cyclic.handleRelease();
							cyclicTouch.active = false;
						}
						break;
					}
				}
			}
			break;
		
		}


		
		for(int i = 0; i < pointers; i++)
		{
			if(throttleTouch.active &&
				motionEvent.getPointerId(i) == throttleTouch.id)
			{
				throttle.handleMotion(
					(int)motionEvent.getX(i), 
					(int)motionEvent.getY(i));
			}
			else
			if(cyclicTouch.active &&
				motionEvent.getPointerId(i) == cyclicTouch.id)
			{
				cyclic.handleMotion(
					(int)motionEvent.getX(i), 
					(int)motionEvent.getY(i));
			}
		}

		return true;
	}

    

    Vector<Container> readouts = new Vector<Container>();
    TouchPoint throttleTouch = new TouchPoint();
    TouchPoint cyclicTouch = new TouchPoint();
    StickPanel throttle;
	StickPanel cyclic;
	Paint paint;

}
