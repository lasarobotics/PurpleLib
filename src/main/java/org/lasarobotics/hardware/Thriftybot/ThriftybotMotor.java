package org.lasarobotics.hardware.Thriftybot;

import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.ctre.PhoenixCANBus;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.thethriftybot.ThriftyNova;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;


public class ThriftybotMotor implements LoggableHardware{

    public static class ID {
        public final String name;
        public final int deviceID;


        public ID(String name, int deviceID) {
            this.name = name;
            this.deviceID = deviceID;
        }
    }
}


