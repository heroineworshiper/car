package com.truck;

import android.graphics.Canvas;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Rect;
import android.graphics.RectF;

public class StickPanel extends Container {

	
	StickPanel(int x, 
		int y,
		int w, 
		int h)
	{
		super(x, y, w, h);
		
		userX = 0;
		userY = 0;
		originX = x + w / 2;
		originY = y + h / 2;
	}
	

	public boolean handlePress(int x, int y)
	{
		if(x >= this.x && 
				y >= this.y && 
				x < this.x + this.w && 
				y < this.y + this.h)
		{
			isActive = true;
			originX = x + w / 2;
			originY = y + h / 2;
			set_origin(x, y);
			return true;
		}
		return false;
	}

	public void handleMotion(int x, int y)
	{
		getUserValue(x, y);
	}

	public void handleRelease()
	{
		originX = x + w / 2;
		originY = y + h / 2;
		userX = 0;
		userY = 0;
		isActive = false;
	}


	public void draw(Canvas g, Paint p)
	{
		int radius = w / 4;
		p.setColor(Truck.settings.foreground);
        p.setStrokeWidth(4);
        
        g.drawRect(new RectF(x, y, x + w, y + h), p);

        float originX = this.originX;
        float originY = this.originY;
        g.drawCircle((float)(originX),  
        		(float)(originY), 
        		(float)radius, 
        		p);

        
        float minDimension = Math.min(w / 2, h / 2);
        float userX = this.userX * minDimension;
        float userY = this.userY * minDimension;
		p.setPathEffect(new DashPathEffect(new float[] {5, 10}, 0));
		g.drawLine(originX, originY, originX + userX, originY - userY, p);
		g.drawCircle((float)(originX + userX),  
        		(float)(originY - userY), 
        		(float)radius, 
        		p);
		p.setPathEffect(null);

        
	}


	void getUserValue(int x, int y)
	{
		float minDimension = Math.min(w / 2, h / 2);
		userX = (float)(x - originX) / minDimension;
		userY = -(float)(y - originY) / minDimension;
	}

	
	
	
	
	
	public void set_origin(int x, int y)
	{
		this.originX = x;
		this.originY = y;
	}
	
	public void set_value(int x, int y)
	{
		this.userX = x;
		this.userY = y;
	}
	
	
	boolean isActive = false;
	// absolute x & y
	int originX, originY;
	// -1 to 1
	float userX, userY;
}
