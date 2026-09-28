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

        // Reserved inbound-to-pad IDs outside the key (0x00-0x0F) / underglow
        // (0x10-0x13) ranges - same 5-byte frame as UpdateLED, just new
        // meanings. Must match firmware's HEARTBEAT_ID/THROTTLE_TELEMETRY_ID/
        // WARP_TELEMETRY_ID exactly.
        private const byte HEARTBEAT_ID = 0x14;
        private const byte THROTTLE_TELEMETRY_ID = 0x15;
        private const byte WARP_TELEMETRY_ID = 0x16;

        private const float HEARTBEAT_INTERVAL = 1.0f; // seconds; must stay well under firmware's HEARTBEAT_TIMEOUT (3.0s)

        private float lastHeartbeatTime = 0f;
        private int lastSentThrottle = -1; // -1 = never sent yet, forces the first send
        private int lastSentWarp = -1;

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

            SendHeartbeat();
            SendTelemetry();
        }

        // Sent on a fixed interval regardless of state change (the one
        // exception to the "only send on diff" rule) - lets the pad show
        // a real CONN/NO CONN indicator instead of guessing.
        void SendHeartbeat()
        {
            if (Time.time - lastHeartbeatTime < HEARTBEAT_INTERVAL)
                return;

            lastHeartbeatTime = Time.time;
            UpdateLED(HEARTBEAT_ID, 0x00, 0x00);
        }

        // Mode-1 OLED telemetry: live throttle % and warp index, diffed
        // against the last value actually sent (same "only on change"
        // pattern as the LED states).
        void SendTelemetry()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            int throttlePct = (int)Mathf.Clamp(vessel.ctrlState.mainThrottle * 100f, 0f, 100f);
            if (throttlePct != lastSentThrottle)
            {
                lastSentThrottle = throttlePct;
                UpdateLED(THROTTLE_TELEMETRY_ID, 0x00, (byte)throttlePct);
            }

            int warpIndex = TimeWarp.CurrentRateIndex;
            if (warpIndex != lastSentWarp)
            {
                lastSentWarp = warpIndex;
                UpdateLED(WARP_TELEMETRY_ID, 0x00, (byte)warpIndex);
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

        // Shared 5-byte outbound frame writer: [0x77][id][state][data][0x44].
        // Used for LED updates as well as HEARTBEAT/telemetry - the name is
        // legacy from when it only sent LED colors.
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