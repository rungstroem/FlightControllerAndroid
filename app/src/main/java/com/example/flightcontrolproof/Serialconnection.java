package com.example.flightcontrolproof;

import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.util.List;

public class Serialconnection {
    UsbManager mUSBMan;
    UsbSerialDriver mSerialDriver;
    UsbDeviceConnection mUSBCon;
    UsbSerialPort mPort;

    private int baudrate;
    private int databits;
    private int TIMEOUT = 10;
    private Context mContext;

    public Serialconnection(int baud, int databit, Context context){
        baudrate = baud;
        databits = databit;
        mContext = context;
        setupSerialDev();
    }
    public void finalize(){
        try {
            mPort.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        mUSBCon.close();    //Unload all USB resources
    }

    public void setupSerialDev(){
        mUSBMan = (UsbManager) mContext.getSystemService(mContext.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(mUSBMan);
        if(availableDrivers.isEmpty()){
            Log.i("Serial","Devices - No devices");
            return;
        }else{
            Log.i("Serial","Devices - Device found");
        }
        mSerialDriver = availableDrivers.get(0);
        mUSBCon = mUSBMan.openDevice(mSerialDriver.getDevice());
        if(mUSBCon == null){
            Log.i("Serial","Open device - No permission");
            return;
        }else{
            Log.i("Serial","Open device - Permission granted");
        }
        mPort= mSerialDriver.getPorts().get(0);
        try{
            mPort.open(mUSBCon);
        }catch(IOException e){
            e.printStackTrace();
            //Do nothing for now
        }
        try {
            mPort.setParameters(baudrate, databits, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
        } catch(IOException e){
            e.printStackTrace();
            //Do nothing for now
        }
    }

    public void rx_data(){
        new Thread(new Runnable() {
            int dataOK = 0;
            byte[] data = null;
            @Override
            public void run() {
                //dataOK = mConnection.bulkTransfer(mEndIN,data,1,TIMEOUT);   // wait until the serial setup is done
                if(dataOK > 0){
                    Log.i("Serial", "Data received - "+data);
                }else{
                    Log.i("Serial", "Data received - No data");
                }
            }
        }).start();
    }

    public void tx_data(int data){
        byte[] conv = new byte[1];
        conv[0] = (byte) (data & 0xFF);
        this.tx_data(conv);
    }
    public void tx_data(double data){
        byte[] conv = new byte[1];
        conv[0] = (byte) ( (int)data & 0xFF);
        this.tx_data(conv);
    }
    public void tx_data(byte[] data){
        try {
            mPort.write(data, TIMEOUT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
