package com.example.flightcontrolproof;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import java.io.IOException;
import java.net.ServerSocket;

public class WifiCommunicationService extends Service {
    Thread wifiThread;
    volatile boolean threadInterrupt = false;

    int portNr = 49190;
    private ServerSocket server;

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
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        //Close the socket
        if(server != null) {
            try{
                server.close();
            }catch (IOException e){
                e.printStackTrace();
            }
        }
        threadInterrupt = true;
        Log.i("SystemState","Wifi communication service shutting down");
    }

    private Runnable WifiCommunication = new Runnable() {
        @Override
        public void run() {
            try {
                server = new ServerSocket(portNr);

            } catch (IOException e) {
                e.printStackTrace();
            }

            while(!threadInterrupt){

            }
            Log.i("SystemState","Wifi communication thread stopped");
        }
    };
}