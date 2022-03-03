package com.example.flightcontrolproof;

public class PIDController {
    private double Kp = 0.0;
    private double Ki = 0.0;
    private double Kd = 0.0;
    private double saturationH = 100;
    private double saturationL = 0;
    private double integrator;
    private double differentiator;
    private double tau = 0.01;  //Guess on differentiator time constant

    private double u;
    private double uUnsaturated;
    private double uSaturated;

    private double error;       //Current error
    private double error_d1;    //Error delayed 1 sample period

    public PIDController(double KP, double KI, double KD){
        this.Kp = KP;
        this.Ki = KI;
        this.Kd = KD;
        error = 0.0;
        error_d1 = 0.0;
        integrator = 0.0;
        differentiator = 0.0;
        u = 0.0;
    }
    public void setSaturation(double satH, double satL){
        saturationH = satH;
        saturationL = satL;
    }

    public void setGains(double KP, double KI, double KD){
        Kp = KP;
        Ki = KI;
        Kd = KD;
    }

    private double saturation(double u){
        if(u>saturationH){
            uSaturated = saturationH;
        }else if(u<saturationL){
            uSaturated = saturationL;
        }else{
            uSaturated = u;
        }
        return uSaturated;
    }

    public double control(double y_c, double y, double Ts, double tau){     //Specific tau constant
        error = y_c - y;
        integrator = integrator + (Ts/2) * (error + error_d1);
        differentiator = (2*tau-Ts)/(2*tau+Ts) * differentiator + 2/(2*tau+Ts) * (error - error_d1);
        error_d1 = error;

        uUnsaturated = (Kp*error + Ki*integrator + Kd*differentiator);
        u = saturation(uUnsaturated);

        if(Ki != 0.0){  //Anti-windup
            integrator = integrator + Ts/Ki * (u-uUnsaturated);
        }

        return u;
    }

    public double control(double y_c, double y, double Ts){         //Standard tau constant
        error = y_c - y;
        integrator = integrator + (Ts/2) * (error + error_d1);
        differentiator = (2*tau-Ts)/(2*tau+Ts) * differentiator + 2/(2*this.tau+Ts) * (error - error_d1);
        error_d1 = error;

        uUnsaturated = (Kp*error + Ki*integrator + Kd*differentiator);
        u = saturation(uUnsaturated);

        if(Ki != 0.0){  //Anti-windup
            integrator = integrator + Ts/Ki * (u-uUnsaturated);
        }

        return u;
    }
}