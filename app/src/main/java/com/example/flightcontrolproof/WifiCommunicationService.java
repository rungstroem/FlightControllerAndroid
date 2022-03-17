package com.example.flightcontrolproof;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.os.IBinder;
import android.provider.ContactsContract;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;

public class WifiCommunicationService extends Service {
    Thread wifiThread;
    volatile boolean threadInterrupt = false;
    volatile boolean comunicationLost = false;

    int portNr = 49190;
    private ServerSocket serverSocket;
    Socket socket;

    public WifiCommunicationService() {
    }

    @Override
    public IBinder onBind(Intent intent) {
        // TODO: Return the communication channel to the service.
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public void onCreate() {
        super.onCreate();

        //Start server thread
        wifiThread = new Thread(WifiCommunication);
        wifiThread.start();
        Log.i("SystemState","Wifi communication service started");
    }


    @Override
    public void onDestroy() {
        super.onDestroy();

        threadInterrupt = true;
        Log.i("SystemState","Wifi communication service shutting down");
    }

    public void sendDataToActivity(int data){
        Intent intent = new Intent("WifiUpdate");
        intent.putExtra("throttleCommand", data);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }
    public void sendDataToAutopilot(int[] data){
        Intent intent = new Intent("WifiUpdate");
        intent.putExtra("commands",data);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    private Runnable WifiCommunication = new Runnable() {
        int i = 0;
        int dataInBuffer;
        int[] dataBuffer = new int[10];
        int[] commLost = new int[10];
        DataInputStream dataIn;
        volatile boolean socketAccept = true;
        volatile boolean readSuccess = true;
        @Override
        public void run() {
            Log.i("SystemState","Wifi communication thread started");
            if(!threadInterrupt && !comunicationLost){
                try {
                    serverSocket = new ServerSocket(portNr);
                    serverSocket.setSoTimeout(5000);    //5s timeout
                    while(!threadInterrupt && !comunicationLost) {
                        Log.i("WifiConnection", "Waiting for connection");
                        socketAccept = true;
                        try {
                            socket = serverSocket.accept();
                            dataIn = new DataInputStream(new DataInputStream(socket.getInputStream()));
                        }catch (SocketTimeoutException te){
                            socketAccept = false;
                        }
                        if(socketAccept) {
                            while (!threadInterrupt && !comunicationLost) {
                                try {
                                    dataInBuffer = dataIn.read();
                                    if (dataInBuffer == -1) {   //Read end-of-line ie no active connection
                                        Log.i("WifiConnection", "Connection lost");
                                        comunicationLost = true;
                                        //sendDataToActivity(0);    //Turn off motor if connection is lost
                                        commLost[0] = 116;  commLost[1] = 0;
                                        sendDataToAutopilot(commLost);
                                        dataIn.close();
                                        socket.close();
                                    } else {
                                        if(dataInBuffer == 120){
                                            i = 0;
                                            sendDataToAutopilot(dataBuffer);
                                        }else {
                                            dataBuffer[i] = dataInBuffer;
                                            i++;
                                        }
                                        Log.i("WifiConnection", "data " + dataInBuffer);
                                        //sendDataToActivity(dataInBuffer);
                                    }
                                }catch (SocketTimeoutException tee){

                                }
                            }
                        }
                    }
                } catch (IOException e) {
                    comunicationLost = true;
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Wifi communication thread stopped");
            return;
        }
    };
}

/*

if(!threadInterrupt && !comunicationLost){
                try {
                    serverSocket = new ServerSocket(portNr);
                    serverSocket.setSoTimeout(5000);    //5s timeout
                    while(!threadInterrupt && !comunicationLost) {
                        Log.i("WifiConnection", "Waiting for connection");
                        socketAccept = true;
                        try {
                            socket = serverSocket.accept();
                            dataIn = new DataInputStream(new DataInputStream(socket.getInputStream()));
                            //DataInputStream dataIn = new DataInputStream(new DataInputStream((socket.getInputStream())));
                        }catch (SocketTimeoutException te){
                            socketAccept = false;
                        }
                        if(socketAccept) {
                            while (!threadInterrupt && !comunicationLost) {
                                readSuccess = true;
                                try {
                                    dataInBuffer = dataIn.read();
                                }catch (SocketTimeoutException tee){
                                    readSuccess = false;
                                }
                                if(readSuccess) {
                                    if (dataInBuffer == -1) {   //Read end-of-line ie no active connection
                                        Log.i("WifiConnection", "Connection lost");
                                        comunicationLost = true;
                                        sendDataToActivity(0);    //Turn off motor if connection is lost
                                        dataIn.close();
                                        socket.close();
                                    } else {
                                        Log.i("WifiConnection", "data " + dataInBuffer);
                                        sendDataToActivity(dataInBuffer);
                                    }
                                }
                            }
                        }
                    }
                } catch (IOException e) {
                    comunicationLost = true;
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Wifi communication thread stopped");
            return;
 */



/*

Log.i("SystemState","Wifi communication thread started");
            while(!threadInterrupt && !comunicationLost){
                try {
                    serverSocket = new ServerSocket(portNr);
                    Log.i("WifiConnection", "Waiting for connection");
                    socket = serverSocket.accept();
                    DataInputStream dataIn = new DataInputStream(new DataInputStream((socket.getInputStream())));
                    while (!threadInterrupt && !comunicationLost) {
                        //dataInBuffer = dataIn.readLine();
                        dataInBuffer = dataIn.read();
                        if (dataInBuffer == -1) {
                            Log.i("WifiConnection", "Connection lost");
                            comunicationLost = true;
                            sendDataToActivity(0);    //Turn off motor if connection is lost
                            dataIn.close();
                            socket.close();
                        } else {
                            Log.i("WifiConnection","data "+dataInBuffer);
                            sendDataToActivity(dataInBuffer);
                        }
                    }
                } catch (IOException e) {
                    comunicationLost = true;
                    e.printStackTrace();
                }
            }
            if(closeSocket){
                try {
                    socket.close();
                }catch(IOException io){

                }
                closeSocket=false;
            }
            Log.i("SystemState","Wifi communication thread stopped");
            return;
 */