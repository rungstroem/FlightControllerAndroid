package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;

public class WifiCommunicationClient extends AppCompatActivity {

    Thread wifiClientThread;
    volatile boolean threadInterrupt = false;
    volatile boolean newData = false;
    volatile boolean communicationLost = false;

    private Socket clientSocket;
    private static int serverPort = 49190;
    private static String serverIP = "192.168.8.121";

    private int PWM = 0;

    SeekBar mSeekBar;
    TextView comStatus;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_wifi_communication_client);

        //Create client thread
        wifiClientThread = new Thread(client);
        wifiClientThread.start();

        mSeekBar = findViewById(R.id.ThrotteCommand);
        mSeekBar.setOnSeekBarChangeListener(seekBarChangeListener);
        comStatus = findViewById(R.id.ConnectionStatus);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        threadInterrupt = true;
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
            newData = true;
        }
    };

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
                            comStatus.setText("Connected to "+serverIP);
                        }
                    });
                    while (!threadInterrupt && !communicationLost) {
                        if (newData) {
                            Log.i("Wifi","Trying to send data");
                            try {
                                dataOut.write(PWM);
                                dataOut.flush();
                                newData = false;
                            }catch (IOException io){
                                Log.i("WifiCommunication","Communication lost");
                                communicationLost = true;
                                dataOut.close();
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
                        comStatus.setText("Connection lost");
                    }
                });
                Log.i("WifiCommuniation","closing socket");
                closeSocket = false;
            }
            Log.i("WifiCommunication","returning from thread");
            return;
        }
    };
}