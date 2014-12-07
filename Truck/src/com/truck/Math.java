package com.truck;

import android.util.Log;

public class Math {

	
	static public void euler_dc(double dst[][], double euler[])
	{
// euler is always level, in this case
 		double	cpsi	= 1;
 		double	cphi	= 1;
 		double	ctheta	= 1;
 
 		double	spsi	= 0;
 		double	sphi	= 0;
 		double	stheta	= 0;




		dst[0][0] = cpsi * ctheta;
		dst[0][1] = spsi * ctheta;
		dst[0][2] = -stheta;

		dst[1][0] = -spsi * cphi + cpsi * stheta * sphi;
		dst[1][1] = cpsi * cphi + spsi * stheta * sphi;
		dst[1][2] = ctheta * sphi;

		dst[2][0] = spsi * sphi + cpsi * stheta * cphi;
		dst[2][1] = -cpsi * sphi + spsi * stheta * cphi;
		dst[2][2] = ctheta * cphi;
	}
	
	
	static public void transpose_matrix(double dst[][], double src[][])
	{
		int i, j;

		if(dst.length != src[0].length || dst[0].length != src.length)
		{
			Log.v("CopterMath", "transpose_matrix: size mismatch");
			return;
		}

		for(i = 0; i < src.length; i++)
		{
			for(j = 0; j < src[i].length; j++)
				dst[j][i] = src[i][j];
		}
	}
	
	static public void multiply_matrix_vector(double dst[], double mat[][], double vec[])
	{
		int i, j;
		int n = mat.length;
		int m = mat[0].length;
	
		if(m != vec.length || dst.length != n)
		{
			Log.v("CopterMath", "multiply_matrix_vector: size mismatch");
			return;
		}
	
		for(i = 0; i < n; i++)
		{
			double s = 0;
			for(j = 0; j < m; j++)
				s += mat[i][j] * vec[j];
			dst[i] = s;
		}
	}
	
	static public double toRad(double angle)
	{
		return angle * 2 * Math.PI / 360;
	}
	
	static public double fromRad(double angle)
	{
		return angle * 360 / Math.PI / 2;
	}
	
	
	static public double angleChange(double oldAngle, double newAngle)
	{
		double angleChange = newAngle - oldAngle;
		if(angleChange > Math.PI) angleChange -= 2 * Math.PI;
		else
		if(angleChange < -Math.PI) angleChange += 2 * Math.PI; 

		
		return angleChange;
		
	}
	
	static public double lowpass(double oldValue, double newValue, double bandwidth)
	{
		return newValue * bandwidth + oldValue * (1.0 - bandwidth);
	}
	
	static public double sqr(double x)
	{
		return x * x;
	}
	
	static public double clamp(double x, double min, double max)
	{
		if(x > max) x = max;
		if(x < min) x = min;
		return x;
	}

	static public int clamp(int x, int min, int max)
	{
		if(x > max) x = max;
		if(x < min) x = min;
		return x;
	}

	static public byte setBit(byte data, int bit)
	{
		data |= 1 << bit;
		return data;
	}
	
	static public boolean bitIsSet(byte data, int bit)
	{
		if((data & (1 << bit)) != 0)
		{
			return true;
		}
		else
			return false;
	}
	

	public static int getChecksum(byte[] buffer, int offset, int bytes)
	{
		int result = 0;
		int size = bytes / 2;

		for(int i = 0; i < size; i++)
		{
			int prev_result = result;
			int byte1 = ((int)buffer[offset + i * 2]) & 0xff;
			int byte2 = ((int)buffer[offset + i * 2 + 1]) & 0xff;
// Not sure if word aligned
			int value = byte1 | (byte2 << 8);
			result += value;
			result &= 0xffff;
// Carry bit
			if(result < prev_result) result++;
			result &= 0xffff;
		}

		int result2 = (result & 0xff) << 8;
		result2 |= (result & 0xff00) >> 8;
		return result2;
	}


	static public int write_int16(byte[] data, int offset, int value)
	{
		data[offset++] = (byte)(value & 0xff);
		data[offset++] = (byte)((value >> 8) & 0xff);
		return offset;
	}
	

    static public int read_uint16(byte[] data, int offset)
	{
    	return ((int)(data[offset] & 0xff)) | ((((int)data[offset + 1]) << 8) & 0xff00);
	}



	static public double max(double x, double y)
	{
		if(x > y) return x;
		return y;
	}

	static public double min(double x, double y)
	{
		if(x < y) return x;
		return y;
	}

	static public int max(int x, int y)
	{
		if(x > y) return x;
		return y;
	}

	static public int min(int x, int y)
	{
		if(x < y) return x;
		return y;
	}

	
	final static int X = 0;
	final static int Y = 1;
	final static int Z = 2;
	final static double PI = 3.141592654;

}
