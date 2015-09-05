package com.truck;

public class TouchPoint {
	public TouchPoint()
	{
		active = false;
	}
	
	boolean identical(int x, int y)
	{
		return java.lang.Math.abs(x - this.x) < THRESHOLD &&
				java.lang.Math.abs(y - this.y) < THRESHOLD;
	}
	
	private static final int THRESHOLD = 5;

	
	int minX, maxX;
	int x, y;
	int id;
	boolean active;
	boolean newActive;

}
