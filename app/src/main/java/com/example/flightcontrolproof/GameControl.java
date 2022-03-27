package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.text.TextUtils;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.concurrent.locks.ReentrantLock;

public class GameControl extends AppCompatActivity {
    ReentrantLock mutex = new ReentrantLock();
    Thread wifiClientThread;
    volatile boolean threadInterrupt = false;
    volatile boolean newData = false;
    volatile boolean communicationLost = false;

    private Socket clientSocket;
    private static int serverPort = 49190;
    private static String serverIP = "192.168.8.121";

    ImageView imViewL;
    ImageView imViewR;

    int xL;
    int yL;
    int xR;
    int yR;

    int screenH;
    int screenW;
    int centerH;
    int centerW;

    EditText ipInput;
    Button inputIPOKButton;

    String dataBuffer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        set_layout();
        setContentView(R.layout.activity_game_control);

        DisplayMetrics mDisplay = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(mDisplay);
        screenH = mDisplay.heightPixels;
        screenW = mDisplay.widthPixels;
        centerH = screenH/4;
        centerW = screenW/2-40;
        Log.i("GameController", "size"+screenW+" "+screenH);

        imViewL = findViewById(R.id.ButtonL);
        imViewR = findViewById(R.id.ButtonR);

        ipInput = findViewById(R.id.inputIP);
        inputIPOKButton = findViewById(R.id.inputIPOKButton);
        inputIPOKButton.setOnClickListener(IPInputListener);

        imViewL.setOnTouchListener(LeftListener);
        imViewR.setOnTouchListener(RightListener);

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        threadInterrupt = true;
    }

    protected void set_layout(){
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        getSupportActionBar().hide(); //Shows the app actionbar "name"
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);
    }

    private Runnable client = new Runnable() {
        volatile boolean closeSocket = true;
        @Override
        public void run() {
            while (!threadInterrupt && !communicationLost) {
                try {
                    clientSocket = new Socket(serverIP, serverPort);
                    DataOutputStream dataOut = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            ipInput.setText("Connected to "+serverIP);
                        }
                    });
                    while (!threadInterrupt && !communicationLost) {
                        if (newData) {
                            Log.i("Wifi","Trying to send data");
                            try {
                                mutex.lock();
                                try {
                                    dataOut.writeBytes(dataBuffer);
                                    dataOut.flush();
                                    newData = false;
                                }catch (IOException io){
                                    Log.i("WifiCommunication","Communication lost");
                                    communicationLost = true;
                                    dataOut.close();
                                }
                            }finally {
                                mutex.unlock();
                            }

                        }
                    }
                } catch (IOException e) {
                    communicationLost = true;
                    e.printStackTrace();
                }
            }
            if(closeSocket){
                try {
                    clientSocket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        ipInput.setText("Connection lost");
                    }
                });
                Log.i("WifiCommuniation","closing socket");
                closeSocket = false;
            }
            Log.i("WifiCommunication","returning from thread");
            return;
        }
    };

    View.OnClickListener IPInputListener = new View.OnClickListener() {
        @Override
        public void onClick(View view) {
            switch (view.getId()) {
                case R.id.inputIPOKButton:
                    if (!TextUtils.isEmpty(ipInput.getText().toString())) {
                        Toast.makeText(getApplicationContext(), "Wifi client started", Toast.LENGTH_SHORT).show();
                        serverIP = ipInput.getText().toString();
                        wifiClientThread = new Thread(client);
                        wifiClientThread.start();
                    } else{
                        serverIP = "192.168.8.101";
                        wifiClientThread = new Thread(client);
                        wifiClientThread.start();
                        Toast.makeText(getApplicationContext(), "Input flight controller IP",Toast.LENGTH_LONG).show();
                    }
                    break;
            }
        }
    };

    View.OnTouchListener RightListener = new View.OnTouchListener() {
        @Override
        public boolean onTouch(View view, MotionEvent motionEvent) {
            switch(motionEvent.getAction()){
                case MotionEvent.ACTION_DOWN:
                    break;
                case MotionEvent.ACTION_MOVE:{
                    xR = ((((int)motionEvent.getX()-centerH)/2)+100)/2;
                    if(xR > 100) xR = 100;
                    if(xR < 1) xR = 1;  //Set to 1 because I don't know if you can send 0 (or NULL) via TCP/IP
                    Log.i("GameController","Right "+xR+" "+yR);
                    try {
                        mutex.lock();
                        dataBuffer = "t"+(char)((xR)&0xFF)+"x";
                        newData = true;
                    }finally {
                        mutex.unlock();
                    }
                }
            }
            return false;
        }
    };

    View.OnTouchListener LeftListener = new View.OnTouchListener() {
        @Override
        public boolean onTouch(View view, MotionEvent motionEvent) {
            switch(motionEvent.getAction()){
                case MotionEvent.ACTION_DOWN:
                    break;
                case MotionEvent.ACTION_MOVE:{
                    xL = ((((int)motionEvent.getX()-centerH)/2)+100)/2; //Set resolution lower!
                    yL = ((((int)motionEvent.getY()-centerW)/2)+100)/2;
                    if(xL > 100) xL = 100;
                    if(xL < 1) xL = 1;  //Set to 1 because I don't know if you can send 0 (or NULL) via TCP/IP
                    if(yL > 100) yL = 100;
                    if(yL < 1) yL = 1;  //Set to 1 because I don't know if you can send 0 (or NULL) via TCP/IP
                    Log.i("GameController","Left "+xL+" "+yL);
                    try {
                        mutex.lock();
                        dataBuffer = "r"+(char)((yL)&0xFF)+(char)((xL)&0xFF)+"x";
                        Log.i("dataBuffer","Data"+dataBuffer);
                        newData = true;
                    }finally {
                        mutex.unlock();
                    }
                }
            }
            return false;
        }
    };
}