package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.Window;
import android.view.WindowManager;
import android.widget.SeekBar;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import java.io.IOException;
import java.util.List;

public class MotorTestActivity extends AppCompatActivity {
    UsbManager mUSBMan;
    UsbSerialDriver mSerialDriver;
    UsbDeviceConnection mUSBCon;
    UsbSerialPort mPort;

    int TIMEOUT = 10;
    private int PWM = 0;

    TextView mTextView;
    TextView mDevText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        set_layout();
        setContentView(R.layout.activity_motor_test);

        mTextView = (TextView) findViewById(R.id.JustATextMT);
        mDevText = (TextView) findViewById(R.id.USBIDMT);

        setupSerialDev();
        SeekBar mSeekBar = findViewById(R.id.motorSpeed);
        mSeekBar.setOnSeekBarChangeListener(seekBarChangeListener);
    }

    SeekBar.OnSeekBarChangeListener seekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
            PWM = i;
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            byte[] test = new byte[2];
            test[0] = (byte) 0xFF;
            test[1] = (byte) (PWM & 0xFF);
            tx_data(test);
        }
    };

    void setupSerialDev(){
        mUSBMan = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(mUSBMan);
        if(availableDrivers.isEmpty()){
            mDevText.setText("No devices");
            return;
        }else{
            mDevText.setText("Device found");
        }
        mSerialDriver = availableDrivers.get(0);
        mUSBCon = mUSBMan.openDevice(mSerialDriver.getDevice());
        if(mUSBCon == null){
            mTextView.setText("No permission");
            return;
        }else{
            mTextView.setText("Permission granted");
        }
        mPort= mSerialDriver.getPorts().get(0);
        try{
            mPort.open(mUSBCon);
        }catch(IOException e){
            e.printStackTrace();
            //Do nothing for now
        }
        try {
            mPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
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
                    mTextView.setText("Hej Kenneth");
                }else{
                    mTextView.setText("No data in buffer");
                }
            }
        }).start();
    }

    public void tx_data(byte[] data){
        try {
            mPort.write(data, TIMEOUT);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    protected void set_layout(){
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        getSupportActionBar().hide(); //Shows the app actionbar "name"
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            mPort.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        //mUSBCon.releaseInterface(mUSBInt);
        mUSBCon.close();    //Unload all USB resources
        //mController.destroy();
    }
}