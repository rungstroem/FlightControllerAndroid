package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.widget.TextView;

import java.util.HashMap;
import java.util.Iterator;

public class USBDevices extends AppCompatActivity {

    private HashMap<String, UsbDevice> mMap;
    private TextView mVendorID;
    private TextView mProductID;

    private UsbManager mUSBMan;
    private UsbDevice mUSBDev;
    private UsbDeviceConnection mUSBCon;
    private UsbInterface mUSBInt;
    private UsbEndpoint mEndpointOut = null;
    private UsbEndpoint mEndpointIn = null;
    int TIMEOUT = 10;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_usbdevices);
        mVendorID = (TextView) findViewById(R.id.VendorID);
        mVendorID.setText("Vendor ID\n");
        mProductID = (TextView) findViewById(R.id.ProductID);
        mProductID.setText("Product ID\n");
        setupUSBDev();
    }

    public void setupUSBDev(){  //Depreciated
        mUSBMan = (UsbManager) getSystemService(Context.USB_SERVICE);
        mMap = mUSBMan.getDeviceList();
        Iterator<UsbDevice> deviceIterator = mMap.values().iterator();
        while(deviceIterator.hasNext()){
            mUSBDev = deviceIterator.next();
        }
        mVendorID.append(String.valueOf(mUSBDev.getVendorId()));
        mProductID.append(String.valueOf(mUSBDev.getProductId()));
        mUSBInt = mUSBDev.getInterface(0);
        mEndpointIn = mUSBInt.getEndpoint(0);   //Endpoint 0 is direction in (into host)
        mEndpointOut = mUSBInt.getEndpoint(1);  //Endpoint 1 is direction out (from host)
        UsbDeviceConnection connection = mUSBMan.openDevice(mUSBDev);
        if(connection != null && connection.claimInterface(mUSBInt, true)) {
            mUSBCon = connection;
        }
    }

    @Override
    protected void onDestroy() {
        mUSBCon.releaseInterface(mUSBInt);
        mUSBCon.close();
        super.onDestroy();
    }
}