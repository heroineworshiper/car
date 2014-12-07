package com.truck;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Calendar;
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
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
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
	}


	boolean initializeBluetooth()
	{
 
		socket = null;
		istream = null;
		ostream = null;
		
		BluetoothAdapter mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		if (mBluetoothAdapter == null) {
	        printAlert("No bluetooth adapter\n");
			return true;
		}
		
		
		
		if (!mBluetoothAdapter.isEnabled()) {
	        printAlert("Bluetooth not enabled\n");
			return true;

		}
		else
		{
			Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
			// If there are paired devices
			if (pairedDevices.size() > 0) {
			    // Loop through paired devices
			    for (BluetoothDevice device : pairedDevices) {
			        // Add the name and address to an array adapter to show in a ListView
			        Log.v("Rover2", "initializeBluetooth " + device.getName());
			        if(device.getName().equals(Settings.bluetooth_id))
			        {
			        	BluetoothSocket tmp = null;
			        	try {
			                tmp = device.createRfcommSocketToServiceRecord(uuid);
			            } catch (IOException e) { }

			        	
//		        		Method m = null;
//						try {
//							m = device.getClass().getMethod(
//									 "createRfcosocket", new Class[] {int.class});
//						} catch (NoSuchMethodException e) {
//							e.printStackTrace();
//						}
//						
//		                 try {
//							tmp = (BluetoothSocket) m.invoke(device, 1);
//						} catch (IllegalArgumentException e) {
//							e.printStackTrace();
//						} catch (IllegalAccessException e) {
//							e.printStackTrace();
//						} catch (InvocationTargetException e) {
//							e.printStackTrace();
//						}

			        	socket = tmp;
			        }
			    }
			}
			
			
			if(socket == null)
			{
				printAlert("No bluetooth device found\n");

				return true;
			}
		}

		
		

	
        try {
            // Connect the device through the socket. This will block
            // until it succeeds or throws an exception
            socket.connect();
        } catch (IOException connectException) {
            // Unable to connect; close the socket and get out
            try {
                socket.close();
            } catch (IOException closeException) 
            { 
            	
            }
            
            connectException.printStackTrace();
//			alert("run: couldn't connect to bluetooth device");
			Log.v("Rover2", "run: connect failed ");
	        printAlert("Couldn't open bluetooth socket\n");
	        return true;
        }

        
        try {
			ostream = socket.getOutputStream();
		} catch (IOException e) {
//			alert("run: couldn't get bluetooth ostream");
			Log.v("Rover2", "run: ostream failed");
			e.printStackTrace();
	        printAlert("Couldn't get bluetooth ostream\n");
			return true;
		}

        
        try {
			istream = socket.getInputStream();
		} catch (IOException e) {
			Log.v("Rover2", "run: istream failed");
	        printAlert("Couldn't get bluetooth istream\n");
			e.printStackTrace();
			return true;
		}
        
    
    	printAlert("Got bluetooth connection\n");

		
		
		return false;
	}
	
	
    public void run() 
    {
    	while(true)
	    {
	    	boolean result = true;
	    	
	    	lostContact = false;
	    	initialized = false;
	    	configStatus = 0;
	    	
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
	   		
	    		while(!lostContact)
	    		{
// get battery voltage
	    			beacon[0] = (byte) 0xff;
	    			beacon[1] = (byte) 0x2d;
	    			beacon[2] = (byte) 0xd4;
	    			beacon[3] = (byte) 0xe5;
	    			
	    			
	    			int chksum = Math.getChecksum(beacon, 
	    					Settings.HEADER_SIZE, 
	    					Settings.GROUND_RADIO_OUT_SIZE - 2);
	    			Math.write_int16(beacon, 
	    					Settings.HEADER_SIZE + Settings.GROUND_RADIO_OUT_SIZE - 2, 
	    					chksum);
	
	    			Calendar cal = Calendar.getInstance();
	    			long nextSentTime = cal.getTimeInMillis();
	    			if(nextSentTime - lastSentTime < 1000 / Settings.BEACON_HZ)
	    			{
	    				long difference = lastSentTime + 1000 / Settings.BEACON_HZ - nextSentTime;
	//    				Log.v("Copter", "run difference=" + difference);
	    				
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
	    					Settings.HEADER_SIZE + Settings.GROUND_RADIO_OUT_SIZE);
	    				ostream.flush();
	    			} catch (IOException e) {
	    				e.printStackTrace();
	    			}
	    			
	    			
	    			// receive beacon
	    		    Callable<Integer> readTask = new Callable<Integer>() {
	    		        @Override
	    		        public Integer call() throws Exception {
	    		            return istream.read(receive_buf, 0, receive_buf.length);
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
						droppedPackets = 0;
						if(lostContact)
						{
							lostContact = false;
							printAlert("Regained contact");
						}
	    		    }
	    		    
	//				Log.v("Copter", "run 2 bytes=" + bytes);
	    			int offset = Settings.HEADER_SIZE;
	    			int chksum2 = 0;
	    			while(offset < bytes)
	    			{
	    				byte c = receive_buf[offset];
	    				if(c == Settings.SYNC_CODE)
	    				{
	    					byte packetType = (byte) (receive_buf[offset + 1] & 0xf0);
	    					
	//    					Log.v("Copter", "run packetType=" + packetType);
	    					
	    					switch(packetType)
	    					{
	    					case Settings.PACKET_CONFIG:
	    						chksum = Math.getChecksum(receive_buf, offset, Settings.CONFIG_PACKET_SIZE - 2);
	    						chksum2 = Math.read_uint16(receive_buf, offset + Settings.CONFIG_PACKET_SIZE - 2);
	    						if(chksum == chksum2)
	    						{
	    							configStatus = receive_buf[offset + 2];
	    							offset += Settings.CONFIG_PACKET_SIZE;
	    						}
	    						else
	    						{
	    							offset = bytes;
	    						}
	    						break;
	    						
	    						
	    					case Settings.PACKET_AZIMUTH:
	    						chksum = Math.getChecksum(receive_buf, offset, Settings.AZIMUTH_PACKET_SIZE - 2);
	    						chksum2 = Math.read_uint16(receive_buf, offset + Settings.AZIMUTH_PACKET_SIZE - 2);
	    						if(chksum == chksum2)
	    						{
	    							azimuth_period = Math.read_uint16(receive_buf, offset + 2);
	    							
	    							int accum = 0;
	    							for(int i = 0; i < Settings.AZIMUTH_SEND_SAMPLES; i++)
	    							{
	    								int value = Math.read_uint16(receive_buf, offset + 2 + 4 + i * 2);
	    								accum += value;
	    							}
	    							accum /= Settings.AZIMUTH_SEND_SAMPLES;
	    							
	    							azimuth_history[azimuth_ptr] = accum;
	    							azimuth_ptr++;
	    							azimuth_ptr &= AZIMUTH_HISTORY_SIZE - 1;
	    								
	    							accum = 0;
	    							for(int i = 0; i < AZIMUTH_HISTORY_SIZE; i++)
	    								accum += azimuth_history[azimuth_ptr];
	    							accum /= AZIMUTH_HISTORY_SIZE;
	    							azimuth_threshold = accum;
	    							
	    							
	    							offset += Settings.AZIMUTH_PACKET_SIZE;
	    						}
	    						else
	    						{
	    							offset = bytes;
	    						}
	    						break;
	    						
	    					case Settings.PACKET_ANALOG:
	    						chksum = Math.getChecksum(receive_buf, offset, Settings.ANALOG_PACKET_SIZE - 2);
	    						chksum2 = Math.read_uint16(receive_buf, offset + Settings.ANALOG_PACKET_SIZE - 2);
	    						if(chksum == chksum2)
	    						{
	    							battery_accum += Math.read_uint16(receive_buf, offset + 2);
	    							total_battery++;
	    							if(total_battery >= Settings.BATTERY_OVERSAMPLE)
	    							{
	    								battery_analog = battery_accum / total_battery;
	    								total_battery = 0;
	    								battery_accum = 0;
	        							if(Settings.battery_analog1 != 0)
	        							{
	        								battery_voltage = (float)battery_analog * 
	        										Settings.battery_v1 /
	        										Settings.battery_analog1;
	        							}
	    							}
	    							
	    							
	    							offset += Settings.ANALOG_PACKET_SIZE;
	    						}
	    						else
	    						{
	    							offset = bytes;
	    						}
	    						break;
	    						
	    						
	    					default:
	    						offset = bytes;
	    						break;
	    					}
	    				}
	    				else
	    				{
	    					break;
	    				}
	    			}
	    		}
	    		
	    		
	    		
	    	}
	    	
	    	if(socket != null)
	    	{
	            try {
	                socket.close();
	                socket = null;
	            } catch (IOException closeException) 
	            { 
	            	
	            }
	    	}
	    	
	    }
    }

    
    void sendConfig()
    {
//    	Log.v("Copter", "sendConfig section=" + currentConfig);
    	
    	int offset = Settings.HEADER_SIZE;
    	beacon[offset++] = Settings.SYNC_CODE;
    	beacon[offset++] = (1 << Settings.CONFIG_VALID_BIT);
    	// next vehicle/packet counter
    	beacon[offset++] = 0;
    	// padding/POV panel
    	beacon[offset++] = 0;
    	beacon[offset++] = (byte) currentConfig;
    	beacon[offset++] = 0;

    	switch(currentConfig)
		{
    	case 1:
    		offset = Math.write_int16(beacon, offset, Settings.MIN_THROTTLE);
    		offset = Math.write_int16(beacon, offset, Settings.MAX_THROTTLE);
    		break;
		}
    	
    	currentConfig++;
    	if(currentConfig >= Settings.CONFIG_SECTIONS)
    		currentConfig = 0;
    }

    void printAlert(String string)
    {
		Message message = Message.obtain(
				WindowBase.handler, 
				2, 
				activity);
		message.getData().putString("text", string);
		WindowBase.handler.sendMessage(message);
	
    }
    
    
    
	static Truck truck;
	static Activity activity;
	static Settings settings;
	
 // the mane connection
    BluetoothSocket socket = null;
	OutputStream ostream = null;
    InputStream istream = null;
    
    byte configStatus = 0;
    int currentConfig = 0;
    int mane_throttle = Settings.MIN_THROTTLE;
    int azimuth_period;
    int azimuth_threshold;
    int azimuth_on;
    int azimuth_off;
    int azimuth_level;
    int azimuth_ptr;
    int battery_accum;
    int total_battery;
    int battery_analog;
    float battery_voltage;
    long lastSentTime;
    boolean initialized = false;
    int droppedPackets = 0;
    boolean lostContact = false;
    
    static final int AZIMUTH_HISTORY_SIZE = 16;
    int [] azimuth_history = new int[AZIMUTH_HISTORY_SIZE];
	byte[] beacon = new byte[Settings.HEADER_SIZE + Settings.GROUND_RADIO_OUT_SIZE];
	byte[] receive_buf = new byte[Settings.RADIO_BUFSIZE];
	ExecutorService executor = Executors.newFixedThreadPool(2);
//	UUID uuid = UUID.randomUUID();
	UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
}
