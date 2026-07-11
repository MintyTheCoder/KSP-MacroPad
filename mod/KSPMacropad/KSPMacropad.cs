using JetBrains.Annotations;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

namespace KSPMacropad
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPMacropad : MonoBehaviour
    {
        SerialPort serialPort = new SerialPort("COM3", 115200);
        private Queue<byte[]> messageQueue = new Queue<byte[]>();
        private readonly object queueLock = new object();

        private byte[] ledStates = new byte[15];

        private bool running = false;

        void Start()
        {
            Debug.Log("[KSPMacropad] KSP Macropad loaded.");
            Thread serialThread = new Thread(ReadSerialLoop);

            serialPort.BaudRate = 115200;
            serialPort.Open();

            serialThread.IsBackground = true;
            running = true;
            serialThread.Start();

            InitializeLEDs();
        }

        //packet structure: [0x44][type: 1 byte][id: 1 byte][value: 2 bytes][0x77]
        void Update()
        {
            byte[] msg = null;
            lock (queueLock)
            {
                if (messageQueue.Count > 0)
                    msg = messageQueue.Dequeue();
            }

            if (msg != null)
            {
                switch (msg[1])
                {
                    case 0x01:
                        Debug.Log("[KSPMacropad] Trigger type: Key");
                        //add the 16 ids for the keys and what to do with them
                        switch (msg[2])
                        {
                            case 0x00:
                                Debug.Log("[KSPMacropad] KEY PRESSED: LAUNCH SEQUENCE");
                                //initialize launch sequence
                                break;
                            case 0x01:
                                Debug.Log("[KSPMacropad] KEY PRESSED: AUTO GRAVITY TURN");
                                //initiate gravity turn
                                break;

                            case 0x02:
                                Debug.Log("[KSPMacropad] KEY PRESSED: CIRCULARIZE");
                                //initiate circularization
                                break;

                            case 0x03:
                                Debug.Log("[KSPMacropad] KEY PRESSED: TIME ACCELERATION TO NXT BURN");
                                //acceklerate time to next burn
                                break;

                            case 0x04:
                                Debug.Log("[KSPMacropad] KEY PRESSED: INTERCEPT CALCULATION");
                                //calculate intercept trajectory for target
                                break;

                            case 0x05:
                                Debug.Log("[KSPMacropad] KEY PRESSED: ORBIT SYNC");
                                //sync orbit with target
                                break;

                            case 0x06:
                                Debug.Log("[KSPMacropad] KEY PRESSED: RENDEZVOUS PREPARATION");
                                //prepare for rendezvous with target
                                break;

                            case 0x07:
                                Debug.Log("[KSPMacropad] KEY PRESSED: DEORBIT BURN");
                                //initiate deorbit burn
                                break;

                            case 0x08:
                                Debug.Log("[KSPMacropad] KEY PRESSED: DOCKING PREP");
                                //prepare for docking with target vessel
                                break;

                            case 0x09:
                                Debug.Log("[KSPMacropad] KEY PRESSED: LANDING PREP");
                                //prepare to land on target body
                                break;

                            case 0x0A:
                                Debug.Log("[KSPMacropad] KEY PRESSED: SUICIDE BURN ARM");
                                //prepare for suicide burn
                                break;

                            case 0x0B:
                                Debug.Log("[KSPMacropad] KEY PRESSED: PRECISION INPUT TOGGLE");
                                //switch encoder mode to precision
                                break;

                            case 0x0C:
                                Debug.Log("[KSPMacropad] KEY PRESSED: TRANSMIT SCIENCE");
                                //transmit all science on board
                                break;

                            case 0x0D:
                                Debug.Log("[KSPMacropad] KEY PRESSED: RESOURCE MONITOR MODE");
                                //switch to resource monitor
                                break;

                            case 0x0E:
                                Debug.Log("[KSPMacropad] KEY PRESSED: AUXILARY MODE TOGGLE");
                                //switch encoder mode to auxilary
                                break;

                            case 0x0F:
                                Debug.Log("[KSPMacropad] KEY PRESSED: AUTOPILOT");
                                //initiate autopilot
                                break;
                        }

                        break;

                    case 0x02:
                        Debug.Log("[KSPMacropad] Trigger type: Encoder");
                        short steps;
                        switch(msg[2])
                        {
                            case 0x11:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 1): ENCDR_LEFT_MD1");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;

                            case 0x12:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 1): ENCDR_RIGHT_MD1");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;

                            case 0x21:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 2): ENCDR_LEFT_MD2");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;

                            case 0x22:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 2): ENCDR_RIGHT_MD2");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;

                            case 0x31:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 3): ENCDR_LEFT_MD3");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;

                            case 0x32:
                                Debug.Log("[KSPMacropad] TURNED ENCODER(mode 3): ENCDR_RIGHT_MD3");
                                steps = (short)((msg[3] << 8) | msg[4]);
                                break;
                        }
                        break;
                }

            }
        }
        

        void ReadSerialLoop()
        {
            while (running)
            {
                if (serialPort.BytesToRead > 0)
                {
                    byte[] buffer = new byte[6];

                    serialPort.Read(buffer, 0, 6);

                    if (buffer[0] != 0x44 || buffer[5] != 0x77)
                    {
                        continue; // Invalid message, skip processing
                    }

                    lock (queueLock)
                    {
                        messageQueue.Enqueue(buffer);
                    }

                }

            }
        }

        void OnDestroy()
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                running = false;
                serialPort.Close();
            }
        }

        //LED packet structure: [0x77][led_id: 1 byte][state: 1 byte][data: 1 byte][0x44]
        void UpdateLED(byte led_id, byte state, byte data)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                byte[] buffer = new byte[5];

                buffer[0] = 0x77;
                buffer[1] = led_id;
                buffer[2] = state;
                buffer[3] = data;
                buffer[4] = 0x44;

                serialPort.Write(buffer, 0, 5);
            }
        }

        void UpdateUnderglow(byte state, byte data)
        {
            for (byte i = 0x10; i < 0x14; i++)
            {
                UpdateLED(i, state, data);
            }
        }

        void InitializeLEDs()
        {
            Array.Clear(ledStates, 0, ledStates.Length);

            for (int i = 0; i < 14; i++)
            {
                UpdateLED((byte)i, 0x00, 0x00);
            }

            UpdateUnderglow(0x00, 0x00);
        }

    }

    
}