/*
 * Phone app for direct drive truck
 * Copyright (C) 2012-2023 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */




package com.truck;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Calendar;
import java.util.Formatter;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.DialogInterface;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

public class Truck extends Thread {
	public Truck(Activity activity)
	{
		this.activity = activity;
	}
	
	
	public static void createObjects(Activity activity)
	{
		if(settings == null)
		{
			settings = new Settings(activity);
			truck = new Truck(activity);
			truck.start();
		}
		
		
    	Log.i("Truck", "createObjects 1 Settings.writeDebug=" + Settings.writeDebug);
		if(Settings.writeDebug)
		{
			debugFile = new File(Settings.DIR + Settings.DEBUG_FILENAME);
			try {
				debugWriter = new BufferedWriter(new FileWriter(debugFile));
			} catch (Exception e) 
			{
    			Log.i("Truck", "createObjects 1 " + e.toString());
    			return;
			}

		}
	}


	boolean initializeBluetooth()
	{
 
		socket = null;
		istream = null;
		ostream = null;


		printAlert("Trying to connect\n");

		BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
	        printAlert("No bluetooth adapter\n");
			return true;
		}
		
		
		BluetoothDevice device = null;
		if (!mBluetoothAdapter.isEnabled()) {
	        printAlert("Bluetooth not enabled\n");
			return true;

		}
		else
		{
			Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
			// Loop through paired devices
			for (int i = 0; i < pairedDevices.size(); i++) {
				device = (BluetoothDevice)pairedDevices.toArray()[i];

//		        Log.i("Truck", "initializeBluetooth " + device.getName());
			    if(device.getName().equals(Settings.bluetooth_id))
			    {
			        try {
			            socket = device.createRfcommSocketToServiceRecord(uuid);
			        } catch (IOException e) 
					{ 
            			e.printStackTrace();
					}

					break;
			    }
			}
			
			
			if(socket == null)
			{
				printAlert("Bluetooth device not paired\n");

				return true;
			}
		}

		
		

	
        try {
            // Connect the device through the socket. This will block
            // until it succeeds or throws an exception
			Log.i("", "Truck.initializeBluetooth: Trying createRfcommSocketToServiceRecord");
            socket.connect();
        } catch (IOException e) {
            e.printStackTrace();
			
			
			
            // Unable to connect; close the socket and get out
            try {
                socket.close();
            } catch (IOException closeException) 
            { 
            	
            }


        	try 
			{
              Log.i("", "Truck.initializeBluetooth: Trying createRfcommSocket");
				socket =(BluetoothSocket) device.getClass().getMethod("createRfcommSocket", new Class[] {int.class}).invoke(device, 1);
				socket.connect();
        	} catch (Exception e2) 
			{
               	e2.printStackTrace();
              	try {
            		socket.close();
              	} catch (IOException e3) 
				{
              	}
				
				
 				socket = null;
				Log.i("", "Truck.initializeBluetooth: connect failed ");
	       		printAlert("Couldn't open bluetooth socket\n");
	     	   	return true;
	       	}
        }

        
        try {
			ostream = socket.getOutputStream();
		} catch (IOException e) {
//			alert("run: couldn't get bluetooth ostream");
			Log.i("Truck", "initializeBluetooth: ostream failed");
			e.printStackTrace();
	        printAlert("Couldn't get bluetooth ostream\n");
			return true;
		}

        
        try {
			istream = socket.getInputStream();
		} catch (IOException e) {
			Log.i("Truck", "initializeBluetooth: istream failed");
	        printAlert("Couldn't get bluetooth istream\n");
			e.printStackTrace();
			return true;
		}
        
    
    	printAlert("Got bluetooth connection\n");

		
		
		return false;
	}
	
	
	public void closeBT()
	{


		if(socket != null)
		{
            try {

				if(istream != null)
				{
					istream.close();
				}

				if(ostream != null) {
					ostream.close();
				}

				socket.close();
            } catch (IOException closeException) 
            { 
            	
            }
		}



		socket = null;
		istream = null;
		ostream = null;
	}
	
    
    static public void confirm(Activity activity, final int command)
    {
		AlertDialog.Builder dialog = new AlertDialog.Builder(activity);
	//	Log.i("Truck", "confirm command=" + command);
        switch(command)
        {
            case RESET_COMMAND:
                dialog.setMessage("Really reset gyro?");
                break;

            case TEST_MOTORS_COMMAND:
                dialog.setMessage("Really test motors?");
                break;
        }
    	
    	dialog.setPositiveButton("Yes", new DialogInterface.OnClickListener()
    	{
            public void onClick(DialogInterface dialog, int which) 
            {
                switch(command)
                {
                    case RESET_COMMAND:
                        Truck.needReset = true;
                        break;

                    case TEST_MOTORS_COMMAND:
                        Truck.testMotors = true;
                        break;
                }
            }
        });
    	dialog.setNegativeButton("No", null);
    	dialog.show();
    }
	
    public void run() 
    {
    	while(true)
	    {
	    	boolean result = true;
	    	
	    	lostContact = false;
	    	initialized = false;
	    	
	    	while(result == true)
	    	{
	    		result = initializeBluetooth();
	    		if(result)
	    		{
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
	    		}
	    	}
	    	
	    	if(result == false)
	    	{
	   		
	    		while(!lostContact && !vehicleChanged)
	    		{
	    			int offset = 0;
	    			beacon[offset++] = (byte) 0xff;
	    			beacon[offset++] = (byte) 0x2d;
	    			beacon[offset++] = (byte) 0xd4;
	    			beacon[offset++] = (byte) 0xe5;

	    			if(needReset)
	    			{
	    				// reset gyro
	    				// size
		    			offset = Math2.write_int16(beacon, offset, 10);
		    			// command
		    			beacon[offset++] = (byte) RESET_COMMAND;
		    			beacon[offset++] = (byte) 0;
		    			
		    			needReset = false;
	    			}
	    			else
	    			if(testMotors)
	    			{
	    				// reset gyro
	    				// size
		    			offset = Math2.write_int16(beacon, offset, 10);
		    			// command
		    			beacon[offset++] = (byte) TEST_MOTORS_COMMAND;
		    			beacon[offset++] = (byte) 0;
		    			
		    			testMotors = false;
	    			}
	    			else
	    			if(needConfig)
	    			{
	                    // size
		                offset += 2;
		                // command
		                beacon[offset++] = (byte) NEW_CONFIG;
                        if(Settings.vehicle == Settings.TRUCK ||
                            Settings.vehicle == Settings.TRUCKCAM)
                        {
                            offset = sendTruckConfig(offset);
                        }
                        else
                        if(Settings.vehicle == Settings.CAR)
                        {
                            offset = sendCarConfig(offset);
                        }

// write the size including the CRC
						Math2.write_int16(beacon, 4, offset + 2);

		    			needConfig = false;
	    			}
	    			else
	    			{
// get battery voltage
// size
		    			Math2.write_int16(beacon, offset, 16);
		    			offset += 2;
// command
		    			beacon[offset++] = (byte) GET_STATUS;
		    			beacon[offset++] = (byte) 0;
// offset = 8
						if(haveControls)
						{
							beacon[offset++] = (byte)1;
							beacon[offset++] = (byte)throttleOut;
							beacon[offset++] = (byte)steeringOut;
							beacon[offset++] = (byte)(needMag ? 1 : 0);
						}
						else
						if(haveControls2)
						{
							beacon[offset++] = (byte)2;
							beacon[offset++] = (byte)0;
							// nested radio packet
							beacon[offset++] = (byte)((reverse ? 1 : 0) |
									(throttleOut2 ? 0 : 2) |
									(steeringOut2 == FAST_LEFT ? 0 : 4) |
									(steeringOut2 == SLOW_LEFT ? 0 : 8) |
									(steeringOut2 == SLOW_RIGHT ? 0 : 16) |
									(steeringOut2 == FAST_RIGHT ? 0 : 32));
							beacon[offset++] = (byte)0;
						}
						else
						{
							beacon[offset++] = (byte)0;
							beacon[offset++] = (byte)0;
							beacon[offset++] = (byte)0;
							beacon[offset++] = (byte)0;
						}

		    			beacon[offset++] = 0;
		    			beacon[offset++] = 0;
	    			}
	    			
	    			
	    			
	    			int checksum = Math2.getChecksum(beacon, 0, offset);
	    			Math2.write_int16(beacon, offset, checksum);
	    			offset += 2;

	    			sendBeacon(offset);

	    			
	    			
	    			
	    			
	    			readPacket();
	    			//printBuffer("run 1", receive_buf, 0, totalReceived);
	    			
	    			int drop_bytes = 0;
	    			for(offset = 0; offset < totalReceived - 4; )
	    			{
	    				drop_bytes = 0;
// got a packet
	    				if(receive_buf[offset] == (byte)0xff &&
	    					receive_buf[offset + 1] == (byte)0x2d &&
	    					receive_buf[offset + 2] == (byte)0xd4 &&
	    					receive_buf[offset + 3] == (byte)0xe5)
	    				{
	    					drop_bytes = offset;
// get size
	    					if(offset < totalReceived - 8)
	    					{
	    						int size = Math2.read_uint16(receive_buf, offset + 4);
	    						if(size + offset > receive_buf.length ||
	    							size < 8)
	    						{
	    							drop_bytes = offset + 6;
	    						}
	    						else
	    						if(offset <= totalReceived - size)
	    						{
									//printBuffer("run 2", receive_buf, 0, totalReceived);

									checksum = Math2.getChecksum(receive_buf, offset, size - 2);
    			    				//Log.i("Truck.run 3", "size=" + size + " checksum1=" + checksum + " checksum2=" + Math2.read_uint16(receive_buf, offset + size - 2));
	    							if(checksum == Math2.read_uint16(receive_buf, offset + size - 2))
	    							{
// packet is intact
		    							switch(receive_buf[offset + 6])
		    							{
		    							    case STATUS_PACKET:
                                            {
                                                int offset2 = offset + 8;
		    								    battery_analog = Math2.read_int32(receive_buf, offset2);
                                                offset2 += 4;
		    								    battery_voltage = Math2.read_float32(receive_buf, offset2);
                                                offset2 += 4;
		    								    gyro_center = Math2.read_int16(receive_buf, offset2);
                                                offset2 += 2;
		    								    gyro_range = Math2.read_uint16(receive_buf, offset2);
                                                offset2 += 2;
		    								    rpm = Math2.read_uint16(receive_buf, offset2);
                                                offset2 += 2;
		    								    current_heading = Math2.read_float32(receive_buf, offset2);
                                                offset2 += 4;
												
												throttleIn = Math2.read_int32(receive_buf, offset2);
												offset2 += 4;
												bluetooth_hz = Math2.read_int16(receive_buf, offset2);
												offset2 += 2;
												radio_hz = Math2.read_int16(receive_buf, offset2);
												offset2 += 2;
                                                
												if(receive_buf[offset2] > 0)
												{
													gotThrottleIn = true;
												}
												else
												{
													gotThrottleIn = false;
												}
												offset2++;
												
												if(Settings.writeDebug &&
													gotThrottleIn)
												{
													gotThrottleIn = false;
													try {
														debugWriter.write(Integer.toString(throttleIn) + "\n");
														debugWriter.flush();
													} catch (IOException e) {
														e.printStackTrace();
													}
												}
												
                                                if(Settings.vehicle == Settings.CAR)
                                                {
                                                    remote_steering = (byte)receive_buf[offset2];
                                                    if(remote_steering < 0)
                                                    {
                                                        remote_steering += 256;
                                                    }
                                                    offset2++;
                                                    remote_throttle = (byte)receive_buf[offset2];
                                                    offset2++;
                                                    if(remote_throttle < 0)
                                                    {
                                                        remote_throttle += 256;
                                                    }
                                                    
                                                    leash_angle = Math2.read_float32(receive_buf, offset2);
                                                    offset2 += 4;
                                                    leash_distance = Math2.read_int16(receive_buf, offset2);
												    offset2 += 2;
                                                }
		    								    break;
                                            }
		    							}
		    							
		    							offset += size;
		    							drop_bytes = offset;
	    							}
	    							else
	    							{
// skip the sync code
	    								offset += 4;
	    								drop_bytes = offset;
	    							}
	    						}
	    						else
	    						{
// get more data
	    							offset = totalReceived;
	    						}
	    					}
	    					else
	    					{
// get more data
	    						offset = totalReceived;
	    					}
	    				}
	    				else
	    				{
	    					drop_bytes++;
	    					offset++;
	    				}

//	    				Log.i("run 2", "drop_bytes=" + drop_bytes + " offset=" + offset);
	    				for(int i = 0; i < totalReceived - drop_bytes; i++)
	    				{
	    					receive_buf[i] = receive_buf[i + drop_bytes];
	    				}
	    				totalReceived -= drop_bytes;
	    				offset -= drop_bytes;
	    				if(offset < 0) offset = 0;
	    				if(totalReceived < 0) totalReceived = 0;
	    			}
	    		}
	    		
	    		
	    		closeBT();
	    	}
	    	
	    }
    }

// Obsolete Tamiya truck
    private int sendTruckConfig(int offset)
    {
        // wheel diameter in mm
		float diameter = Settings.getFileFloat("DIAMETER")[0];
	    // send config
		beacon[offset++] = (byte) 1;

		beacon[offset++] = (byte) (Settings.getFileFloat("VISION_BANDWIDTH")[0] * 255);
		beacon[offset++] = (byte) (Settings.getFileFloat("PATH_CENTER")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("BOTTOM_CENTER")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("ENABLE_VISION")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("ENABLE_MAG")[0]);
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("MANUAL_OVERRIDE_DELAY")[0]));


		beacon[offset++] = (byte) (Settings.getFileFloat("MID_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MID_THROTTLE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MAX_THROTTLE_FWD")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MAX_THROTTLE_REV")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_THROTTLE_FWD")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_THROTTLE_REV")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("THROTTLE_BASE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("THROTTLE_REVERSE_BASE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MAX_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("AUTO_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("AUTO_THROTTLE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("RPM_DV_SIZE")[0]);
//		    			beacon[offset++] = (byte) (Settings.getFileFloat("PATH_DX_SIZE")[0]);

		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("GYRO_CENTER_MAX")[0]));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("MAX_GYRO_DRIFT")[0] * 256));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("ANGLE_TO_GYRO")[0]));
		beacon[offset++] = (byte) (Settings.getFileFloat("GYRO_BANDWIDTH")[0]);
		beacon[offset++] = (byte)(Settings.getFileFloat("D_BANDWIDTH")[0]);



		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("THROTTLE_RAMP_DELAY")[0]));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("THROTTLE_RAMP_STEP")[0]));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("PID_DOWNSAMPLE")[0]));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("STEERING_STEP_DELAY")[0]));



		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("BATTERY_ANALOG")[0]));

        //float targetPace = Settings.getFileFloat("TARGET_PACE")[0];
		float targetPace = Settings.targetPace;
        if(targetPace > 0.001)
        {
            // meters per minute/wheel circumference in meters
            offset = Math2.write_int16(beacon, offset, paceToRPM(targetPace, diameter));
//Log.i("Truck", "run targetRPM=" + targetRpm);
        }
        else
        {
            offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("TARGET_RPM")[0]));
        }


		float targetReversePace = Settings.getFileFloat("TARGET_REVERSE_PACE")[0];
		if(targetPace > 0.001)
		{
			// meters per minute/wheel circumference in meters
			offset = Math2.write_int16(beacon, offset, paceToRPM(targetReversePace, diameter));
//Log.i("Truck", "run targetRPM=" + targetRpm);
		}
		else
		{
			offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("TARGET_REVERSE_RPM")[0]));
		}



		offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("BATTERY_V0")[0]);
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("STEERING_STEP")[0]));
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("STEERING_OVERSHOOT")[0]));

//						offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("STICTION_THRESHOLD")[0]);
//						offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("STICTION_AMOUNT")[0]);




// 						offset = Math2.write_int16(beacon, offset, mag_x_max);
// 						offset = Math2.write_int16(beacon, offset, mag_y_max);
// 						offset = Math2.write_int16(beacon, offset, mag_z_max);
// 						offset = Math2.write_int16(beacon, offset, mag_x_min);
// 						offset = Math2.write_int16(beacon, offset, mag_y_min);
// 						offset = Math2.write_int16(beacon, offset, mag_z_min);






		float pid[] = Settings.getFileFloat("STEERING_PID");
		offset = writePid(offset, pid);

		pid = Settings.getFileFloat("RPM_PID");
		offset = writePid(offset, pid);
        return offset;
    }



// 3D printed truck
    private int sendCarConfig(int offset)
    {
        // wheel diameter in mm
		float diameter = Settings.getFileFloat("DIAMETER")[0];
	    // send config
		beacon[offset++] = (byte) 1;


		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_THROTTLE_FWD")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_THROTTLE_REV")[0]);

		beacon[offset++] = (byte) (Settings.getFileFloat("THROTTLE_BASE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("THROTTLE_REVERSE_BASE")[0]);

// servo PWM
		beacon[offset++] = (byte) (Settings.getFileFloat("MID_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MAX_STEERING")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("MIN_STEERING")[0]);

		beacon[offset++] = (byte) (Settings.getFileFloat("RPM_DV_SIZE")[0]);

		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("GYRO_CENTER_MAX")[0]));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("MAX_GYRO_DRIFT")[0] * 256));
		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("ANGLE_TO_GYRO")[0]));
		beacon[offset++] = (byte) (Settings.getFileFloat("GYRO_BANDWIDTH")[0]);
		beacon[offset++] = (byte)(Settings.getFileFloat("D_BANDWIDTH")[0]);

// remote control ADC
        beacon[offset++] = (byte)(Settings.getFileFloat("STEERING_ADC_CENTER")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("STEERING_ADC_DEADBAND")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("STEERING_ADC_MAX")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("STEERING_ADC_MIN")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("THROTTLE_ADC_CENTER")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("THROTTLE_ADC_DEADBAND")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("THROTTLE_ADC_MAX")[0]);
        beacon[offset++] = (byte)(Settings.getFileFloat("THROTTLE_ADC_MIN")[0]);



		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("PID_DOWNSAMPLE")[0]));



		offset = Math2.write_int16(beacon, offset, (int) (Settings.getFileFloat("BATTERY_ANALOG")[0]));

// base meters per minute/wheel circumference in meters
        offset = Math2.write_int16(beacon, offset, paceToRPM(Settings.targetPace, diameter));

		float targetReversePace = Settings.getFileFloat("TARGET_REVERSE_PACE")[0];
// meters per minute/wheel circumference in meters
		offset = Math2.write_int16(beacon, offset, paceToRPM(targetReversePace, diameter));


// wheel diameter
        beacon[offset++] = (byte)diameter;
        

		offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("BATTERY_V0")[0]);
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("STEERING_STEP")[0]));
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("MAX_STEERING_STEP")[0]));
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("STEERING_OVERSHOOT")[0]));

		float pid[] = Settings.getFileFloat("STEERING_PID");
		offset = writePid(offset, pid);

		pid = Settings.getFileFloat("RPM_PID");
		offset = writePid(offset, pid);


// leash
        beacon[offset++] = (byte)(Settings.getFileFloat("LEASH_DISTANCE0")[0]);;
		offset = Math2.write_float32(beacon, offset, paceToRPM(Settings.getFileFloat("LEASH_SPEED0")[0], diameter));
		offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("LEASH_SPEED_TO_DISTANCE")[0]);
		offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("LEASH_MAX_SPEED")[0]);
		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("LEASH_CENTER")[0]));
		offset = Math2.write_float32(beacon, offset, Settings.getFileFloat("LEASH_X_OFFSET")[0]);
//		offset = Math2.write_float32(beacon, offset, (float)Math2.toRad(Settings.getFileFloat("LEASH_MAX_ANGLE")[0]));
		beacon[offset++] = (byte) (Settings.getFileFloat("LEASH_D_SIZE")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("LEASH_I_LIMIT")[0]);
		beacon[offset++] = (byte) (Settings.getFileFloat("LEASH_D_LIMIT")[0]);

        pid = Settings.getFileFloat("LEASH_PID");
        offset = writePid(offset, pid);
        return offset;
    }

	private int paceToRPM(float targetPace, float diameter) {
        final float M_TO_MI = (float)1609.0;
		return (int) (M_TO_MI / targetPace / (diameter * Math2.PI / 1000));
	}


	private int writePid(int offset, float[] pid) {
		for(int i = 0; i < 4; i++)
		{
			if(i < pid.length)
			{
				Math2.write_float32(beacon, offset, pid[i]);
			}
			else
			{
				Math2.write_float32(beacon, offset, 0);
			}
			
			offset += 4;
		}
		return offset;
	}


	private void sendBeacon(int offset) {
		Calendar cal = Calendar.getInstance();
		long nextSentTime = cal.getTimeInMillis();
		if(nextSentTime - lastSentTime < 1000 / Settings.BEACON_HZ)
		{
			long difference = lastSentTime + 1000 / Settings.BEACON_HZ - nextSentTime;
//    				Log.i("Copter", "run difference=" + difference);
			
			try {
				Thread.sleep(difference);
			} catch (InterruptedException e) {
				return;
			}

		}
		lastSentTime = nextSentTime;
		
		// send beacon
		try {
			ostream.write(beacon, 
				0, 
				offset);
			ostream.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}


	private int readPacket() 
	{
		if(totalReceived >= receive_buf.length)
		{
			printAlert("Receive buffer full");
// reset the buffer
			totalReceived = 0;
		}
		
		// receive response
		Callable<Integer> readTask = new Callable<Integer>() {
		    @Override
		    public Integer call() throws Exception {
		        return istream.read(receive_buf, totalReceived, receive_buf.length - totalReceived);
		    }
		};
		
		int bytes = 0;
		Future<Integer> future = executor.submit(readTask);
		try {
			bytes = future.get(1000 / Settings.BEACON_HZ, TimeUnit.MILLISECONDS);
		} catch (Exception e) {
		}
		
		if(bytes == 0)
		{
			droppedPackets++;
			if(droppedPackets >= Settings.BEACON_HZ &&
				initialized)
			{
				droppedPackets = 0;
				lostContact = true;
				printAlert("Lost contact");
			}
		}
		else
		{
			totalReceived += bytes;
			droppedPackets = 0;
			if(lostContact)
			{
				lostContact = false;
				printAlert("Regained contact");
			}
		}
		return bytes;
	}

	static public void printBuffer(String string, byte[] buffer, int offset, int bytes)
	{
		StringBuilder sb = new StringBuilder();
		Formatter formatter = new Formatter(sb);
		for(int i = 0; i < bytes; i++) 
		{
		formatter.format("%02x ", buffer[i + offset]);
		}
		Log.i(string, sb.toString());
	}

    

    void printAlert(String string)
    {
    	synchronized(this)
    	{
    		Settings.message = string;
    		Settings.haveMessage = true;
    	}
    }
    
    
    
	static Truck truck;
	static Activity activity;
	static Settings settings;
	
 // the mane connection
    BluetoothSocket socket = null;
	OutputStream ostream = null;
    InputStream istream = null;

// commands we send to the vehicle
	static final int GET_STATUS = 0;
	static final int RESET_COMMAND = 1;
	static final int NEW_CONFIG = 2;
	static final int TEST_MOTORS_COMMAND = 3;

// packets we get back from the vehicle
	static final int STATUS_PACKET = 0;
	
    
    static int battery_analog;
    static float battery_voltage;
    static int gyro_center;
    static int gyro_range;
    static int remote_steering;
    static int remote_throttle;
    static float current_heading;
    
    static float leash_angle;
    static int leash_distance;
    
//	static float heading_feedback;
//	static float power;
	static int rpm;
	static int radio_hz, bluetooth_hz;
// 	static int mag_x_min, mag_x_max;
// 	static int mag_y_min, mag_y_max;
// 	static int mag_z_min, mag_z_max;
// 	static int vanish_x, vanish_y, bottom_x;
    
    static boolean needReset = false;
    static boolean testMotors = false;
    static boolean needConfig = false;
//    static boolean needSaveConfig = false;
	static boolean needMag = false;

	// full manual controls
// throttle to send -127 - 127
    static int throttleOut = 0;
// throttle PWM received
    static int throttleIn = 0;
	static boolean gotThrottleIn = false;
// steering to send -127 - 127
    static int steeringOut = 0;
// steering to receive -127 - 127
//    static int steeringIn = 0;
// have fully manual control values to send from the drive window
	static boolean haveControls = false;


	// semi automatic controls
	static boolean throttleOut2 = false;
	static int NO_STEERING = 0;
	static int FAST_LEFT = 1;
	static int SLOW_LEFT = 2;
	static int SLOW_RIGHT = 3;
	static int FAST_RIGHT = 4;
	static int steeringOut2 = NO_STEERING;
	static boolean reverse = false;
	// have semi automatic control values to send from the drive2 window
	static boolean haveControls2 = false;

    
    long lastSentTime;
    boolean initialized = false;
    int droppedPackets = 0;
    boolean lostContact = false;
	boolean vehicleChanged = false;

	byte[] beacon = new byte[Settings.RADIO_BUFSIZE];
	byte[] receive_buf = new byte[Settings.RADIO_BUFSIZE];
	int totalReceived = 0;
	ExecutorService executor = Executors.newFixedThreadPool(2);
//	UUID uuid = UUID.randomUUID();
	UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

// write throttle PWM to a file
	static File debugFile = null;
	static BufferedWriter debugWriter = null;
}
