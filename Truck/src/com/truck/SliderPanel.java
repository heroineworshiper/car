package com.truck;

import java.util.Formatter;

import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Path;
import android.graphics.Rect;
import android.graphics.RectF;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;



// throttle slider for user input
public class SliderPanel extends Container {
	SliderPanel(int x, 
		int y,
		int w, 
		int h,
		int valueH)
	{
		super(x, y, w, h);
		this.valueH = valueH;
	}

	public boolean handlePress(int x, int y)
	{
		if(x >= this.x && 
				y >= this.y && 
				x < this.x + this.w && 
				y < this.y + this.h)
		{
			isActive = true;
			getUserValue(x, y);
			return true;
		}
		return false;
	}

	public void handleMotion(int x, int y)
	{
		getUserValue(x, y);
		set_value(userValue);
	}

	public void handleRelease()
	{
		isActive = false;
	}

	public boolean onTouch(View view, MotionEvent motionEvent) 
	{
		int pointers = motionEvent.getPointerCount();
		
		
		Log.v("SliderPanel", "onTouch pointers=" + 
				pointers + 
				" action=" +
				motionEvent.getAction());
		
		for(int i = 0; i < pointers; i++)
		{
		
		
			
			int x = (int) motionEvent.getX(i);
			int y = (int) motionEvent.getY(i);
			
			if(!isActive)
			{
				if((motionEvent.getAction() == MotionEvent.ACTION_DOWN ||
						motionEvent.getAction() == MotionEvent.ACTION_POINTER_2_DOWN) &&
						x >= this.x && 
						y >= this.y && 
						x < this.x + this.w && 
						y < this.y + this.h)
				{
					isActive = true;
					getUserValue(x, y);
					return true;
				}
			}
			else
			{
				if(motionEvent.getAction() == MotionEvent.ACTION_UP)
				{
					isActive = false;
					return true;
				}
				else
				{
					getUserValue(x, y);
					return true;
				}
			}
		}
		
		return false;
	
	}
	
	
	void getUserValue(int x, int y)
	{
		userValue = -(float)((((float)y - this.y - valueH / 2) / (rectH - valueH)) * 2.0 - 1.0);
		if(userValue > (float)1.0) userValue = (float)1.0;
		if(userValue < (float)-1.0) userValue = (float)-1.0;
//			Log.v("Slider", "getUserValue " + userValue);
	}

	public void draw(Canvas g, Paint p)
	{
		p.setColor(Truck.settings.foreground);
        p.setStrokeWidth(1);
		p.setStyle(Style.FILL);
		
//			p.setTypeface(Copter.settings.small_font);
//			p.setTextSize(Copter.settings.small_font_size);
//			StringBuilder sb = new StringBuilder();
//			Formatter formatter = new Formatter(sb);
//			Rect textSize = new Rect();
//			sb.delete(0, sb.length());
//			formatter.format("%.02f", value);
//			String string = formatter.toString();
//			p.getTextBounds(string,
//	           0,
//	           string.length(),
//	           textSize);
//
//			rectH = h - textSize.height();
//			
//			g.drawText(string, x + w / 2 - textSize.width() / 2, h, p);

		rectH = h;
		p.setStyle(Style.STROKE);
		final int lineWidth = 4;
		final int valueW = w - lineWidth;
		int valueY = (int) (y + rectH / 2 - (value * (rectH - valueH) / 2) - valueH / 2);
		p.setStrokeWidth(lineWidth);
		g.drawRect(new RectF(x, y, x + w, y + rectH), p);

		p.setStrokeWidth(lineWidth);
		g.drawRect(new Rect(x + lineWidth / 2, valueY, x + lineWidth / 2 + valueW, valueY + valueH), p);
		
		
//			if(isActive)
		{
			valueY = (int) (y + rectH / 2 - (userValue * (rectH - valueH) / 2) - valueH / 2);
			p.setPathEffect(new DashPathEffect(new float[] {5, 10}, 0));
			g.drawRect(new Rect(x + lineWidth / 2, valueY, x + lineWidth / 2 + valueW, valueY + valueH), p);
			
			p.setPathEffect(null);
			
		}
//			Log.v("Slider", "draw " + valueY);
	}

	public void set_value(float value)
	{
		this.value = value;
	}

	// value from telemetry
	float value = -1;
	// value from touch event
	float userValue = -1;
	int rectH = 0;
	int valueH = 100;
	boolean isActive = false;
}
