using JetBrains.Annotations;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using UnityEngine;
using KSP.UI.Screens; // StageManager - namespace unverified, no KSP install to check against

namespace KSPMacropad
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPMacropad : MonoBehaviour
    {
        SerialPort serialPort = new SerialPort("COM3", 115200);
        private Queue<byte[]> messageQueue = new Queue<byte[]>();
        private readonly object queueLock = new object();

        // Every key that actually has LED states defined (see LEDStates.cs /
        // STATE_COLORS on the firmware side). Excludes 0x0B (PRECISION INPUT
        // TOGGLE) and 0x0E (AUX MODE TOGGLE) - both onboard-only, no LED
        // states exist for them. NOTE: an earlier version of this file (and
        // the project doc) wrongly said 0x08 (DOCKING PREP) was the excluded
        // one instead of 0x0B - DOCK does have LED states, it's tracked here.
        private static readonly byte[] TrackedKeyIds =
        {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0A, 0x0C, 0x0D, 0x0F
        };

        // key_id -> index into ledStates. Index 14 (== TrackedKeyIds.Length)
        // is the shared underglow slot, matching the 14+1=15 sizing below.
        private static readonly Dictionary<byte, int> KeyIdToStateIndex = BuildKeyIdIndex();
        private static Dictionary<byte, int> BuildKeyIdIndex()
        {
            var map = new Dictionary<byte, int>();
            for (int i = 0; i < TrackedKeyIds.Length; i++)
                map[TrackedKeyIds[i]] = i;
            return map;
        }

        private byte[] ledStates = new byte[TrackedKeyIds.Length + 1];

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

        private bool suicideBurnArmed = false;
        private bool resourceMonitorPanelOpen = false;

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
                                DoLaunchSequence();
                                break;
                            case 0x01:
                                Debug.Log("[KSPMacropad] KEY PRESSED: AUTO GRAVITY TURN");
                                //initiate gravity turn - HARD: continuous closed-loop pitch control, not yet implemented
                                break;

                            case 0x02:
                                Debug.Log("[KSPMacropad] KEY PRESSED: CIRCULARIZE");
                                //initiate circularization - HARD: maneuver node burn vector math, not yet implemented
                                break;

                            case 0x03:
                                Debug.Log("[KSPMacropad] KEY PRESSED: TIME ACCELERATION TO NXT BURN");
                                DoTimeAccelToNextEvent();
                                break;

                            case 0x04:
                                Debug.Log("[KSPMacropad] KEY PRESSED: INTERCEPT CALCULATION");
                                //calculate intercept trajectory - HARD: not yet implemented
                                break;

                            case 0x05:
                                Debug.Log("[KSPMacropad] KEY PRESSED: ORBIT SYNC");
                                //sync orbit with target - HARD: not yet implemented
                                break;

                            case 0x06:
                                Debug.Log("[KSPMacropad] KEY PRESSED: RENDEZVOUS PREPARATION");
                                //prepare for rendezvous - HARD: not yet implemented
                                break;

                            case 0x07:
                                Debug.Log("[KSPMacropad] KEY PRESSED: DEORBIT BURN");
                                //initiate deorbit burn - HARD: maneuver node burn vector math, not yet implemented
                                break;

                            case 0x08:
                                Debug.Log("[KSPMacropad] KEY PRESSED: DOCKING PREP");
                                DoDockingPrep();
                                break;

                            case 0x09:
                                Debug.Log("[KSPMacropad] KEY PRESSED: LANDING PREP");
                                DoLandingPrep();
                                break;

                            case 0x0A:
                                Debug.Log("[KSPMacropad] KEY PRESSED: SUICIDE BURN ARM");
                                DoSuicideBurnArm();
                                break;

                            case 0x0B:
                                Debug.Log("[KSPMacropad] KEY PRESSED: PRECISION INPUT TOGGLE");
                                //switch encoder mode to precision
                                break;

                            case 0x0C:
                                Debug.Log("[KSPMacropad] KEY PRESSED: TRANSMIT SCIENCE");
                                DoTransmitScience();
                                break;

                            case 0x0D:
                                Debug.Log("[KSPMacropad] KEY PRESSED: RESOURCE MONITOR MODE");
                                DoResourceMonitorToggle();
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
            CheckLEDStates();
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

            // FIX: this used to be `for (i = 0; i < 14; i++) UpdateLED((byte)i, ...)`,
            // which sent raw IDs 0x00-0x0D - wrongly touching 0x0B (no LED
            // states exist for it) and never touching 0x0F (AUTOPILOT, which
            // does have LED states). Iterating the real tracked-key list instead.
            foreach (byte keyId in TrackedKeyIds)
            {
                UpdateLED(keyId, 0x00, 0x00);
            }

            UpdateUnderglow(0x00, 0x00);
        }

        // Only sends an update when the resolved state actually differs from
        // what was last sent for that key - same "diff, don't spam" pattern
        // as the rest of the LED protocol.
        void SetLEDState(byte keyId, byte state, byte data)
        {
            int idx = KeyIdToStateIndex[keyId];
            if (ledStates[idx] != state)
            {
                ledStates[idx] = state;
                UpdateLED(keyId, state, data);
            }
        }

        void SetUnderglowState(byte state)
        {
            int idx = TrackedKeyIds.Length; // reserved shared slot
            if (ledStates[idx] != state)
            {
                ledStates[idx] = state;
                UpdateUnderglow(state, 0x00);
            }
        }

        // Only RESOURCE MONITOR and UNDERGLOW are implemented here - both are
        // pure "read current game state, map to a color" checks. Every other
        // tracked key's non-IDLE states (CIRC_CALCULATING vs CIRC_WARPING,
        // LAUNCH_EXECUTING, etc.) depend on bookkeeping from inside that
        // macro's own execution (is a burn in progress, is a node planned) -
        // that doesn't exist yet since those macro bodies are still
        // Debug.Log stubs above. They stay at whatever InitializeLEDs set
        // (IDLE) until the macros themselves are written.
        //
        // NOT verified against a real KSP install/compile (no KSP on this
        // dev machine per the project's usual workflow) - the resource
        // iteration API in particular should be checked against the actual
        // game before trusting this as-is.
        void CheckLEDStates()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;

            if (vessel == null)
            {
                SetLEDState(0x0D, LEDStates.RESOURCE_IDLE, 0x00);
                SetUnderglowState(LEDStates.UNDERGLOW_CONNECTED_NOVESSEL);
                return;
            }

            // --- RESOURCE MONITOR (0x0D) ---
            // Thresholds below are placeholders - pick numbers that feel
            // right in-game, these haven't been tuned against real play.
            // RESOURCE_DEPLETING intentionally not set: distinguishing "low"
            // from "actively draining fast" needs a rate-of-change check
            // (comparing fraction across frames), not just a snapshot -
            // left for later.
            double resourceFraction = GetLowestResourceFraction(vessel);
            byte resourceState;
            if (resourceFraction >= 0.50) resourceState = LEDStates.RESOURCE_NOMINAL;
            else if (resourceFraction >= 0.25) resourceState = LEDStates.RESOURCE_LOW;
            else if (resourceFraction >= 0.10) resourceState = LEDStates.RESOURCE_VERYLOW;
            else resourceState = LEDStates.RESOURCE_CRITICAL;
            SetLEDState(0x0D, resourceState, 0x00);

            // --- UNDERGLOW ---
            byte underglowState;
            switch (vessel.situation)
            {
                case Vessel.Situations.PRELAUNCH:
                    underglowState = LEDStates.UNDERGLOW_LAUNCHPAD;
                    break;
                case Vessel.Situations.LANDED:
                case Vessel.Situations.SPLASHED:
                    underglowState = LEDStates.UNDERGLOW_LANDING;
                    break;
                case Vessel.Situations.ORBITING:
                case Vessel.Situations.ESCAPING:
                    underglowState = LEDStates.UNDERGLOW_STABLE_ORBIT;
                    break;
                default: // FLYING, SUB_ORBITAL, DOCKED
                    underglowState = LEDStates.UNDERGLOW_ASCENT;
                    break;
            }

            if (TimeWarp.CurrentRateIndex > 0 && TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
                underglowState = LEDStates.UNDERGLOW_TIMEWARP; // overrides situation while warping

            if (resourceState == LEDStates.RESOURCE_CRITICAL)
                underglowState = LEDStates.UNDERGLOW_CRITICAL_RESOURCE; // overrides everything else

            SetUnderglowState(underglowState);
        }

        // NOTE: unverified against the real KSP resource API - GetActiveResources()
        // signature/behavior should be double-checked once KSP is available to test.
        double GetLowestResourceFraction(Vessel vessel)
        {
            double lowest = 1.0;
            bool any = false;

            foreach (var resource in vessel.GetActiveResources())
            {
                if (resource.maxAmount <= 0)
                    continue;

                any = true;
                double frac = resource.amount / resource.maxAmount;
                if (frac < lowest)
                    lowest = frac;
            }

            return any ? lowest : 1.0;
        }

        // ------------------------------------------------------------------
        // Macro bodies - the "simple" batch: one-shot action-group toggles,
        // no maneuver-node vector math or continuous closed-loop control.
        // NONE OF THIS IS COMPILED OR TESTED against a real KSP install -
        // written from known KSP modding API patterns, but exact method/
        // namespace names (StageManager vs Staging, Autopilot call order,
        // science API specifics) should be checked once KSP is on hand.
        // ------------------------------------------------------------------

        void DoLaunchSequence()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            FlightInputHandler.state.mainThrottle = 1f;

            if (!vessel.ActionGroups[KSPActionGroup.SAS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.SAS);

            StageManager.ActivateNextStage(); // UNVERIFIED: class name may be `Staging` in some KSP versions

            SetLEDState(0x00, LEDStates.LAUNCH_EXECUTING, 0x00);
            // LAUNCH_COMPLETE is never set here - "launch complete" isn't a
            // single event, it needs a definition (reached target apoapsis?
            // left the atmosphere?) that hasn't been decided yet.
        }

        // Finds the earliest of (next maneuver node UT, next SOI-change UT)
        // and warps to it. TimeWarp.WarpTo() auto-stops on arrival by itself,
        // so no separate "stop warping" call is needed. No burn-direction
        // risk here since this only controls time, not a burn vector.
        void DoTimeAccelToNextEvent()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            double currentUT = Planetarium.GetUniversalTime();
            double? nextEventUT = null;

            if (vessel.patchedConicSolver != null && vessel.patchedConicSolver.maneuverNodes.Count > 0)
                nextEventUT = vessel.patchedConicSolver.maneuverNodes[0].UT;

            if (vessel.orbit.UTsoi > currentUT)
            {
                if (!nextEventUT.HasValue || vessel.orbit.UTsoi < nextEventUT.Value)
                    nextEventUT = vessel.orbit.UTsoi;
            }

            if (nextEventUT.HasValue && nextEventUT.Value > currentUT)
            {
                SetLEDState(0x03, LEDStates.TIMEACCEL_WARPING, 0x00);
                TimeWarp.fetch.WarpTo(nextEventUT.Value);
                // KNOWN GAP: nothing currently resets this LED back to IDLE
                // once the warp completes - CheckLEDStates doesn't watch
                // this key yet. Will show WARPING until something else
                // changes it.
            }
            else
            {
                SetLEDState(0x03, LEDStates.TIMEACCEL_UNAVAILABLE, 0x00);
            }
        }

        void DoDockingPrep()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (!vessel.ActionGroups[KSPActionGroup.RCS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.RCS);

            if (!vessel.ActionGroups[KSPActionGroup.SAS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.SAS);

            if (FlightGlobals.fetch.VesselTarget != null)
            {
                vessel.Autopilot.Enabled = true;
                vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Target);
                SetLEDState(0x08, LEDStates.DOCK_TARGET_ACQUIRED, 0x00);
            }
            else
            {
                SetLEDState(0x08, LEDStates.DOCK_ACTIVE, 0x00);
            }
            // DOCK_READY is not set here - needs a distance/closing-velocity
            // check against the target, which belongs in a continuous
            // monitor (CheckLEDStates), not this one-shot key handler.
        }

        void DoLandingPrep()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (!vessel.ActionGroups[KSPActionGroup.SAS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.SAS);

            if (!vessel.ActionGroups[KSPActionGroup.RCS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.RCS);

            if (!vessel.ActionGroups[KSPActionGroup.Gear])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.Gear);

            FlightInputHandler.state.mainThrottle = 0f;

            SetLEDState(0x09, LEDStates.LANDING_CONFIGURING, 0x00);
            // LANDING_COMPLETE isn't set here - that's `vessel.Landed`
            // going true, which is a continuous-check concern, not a
            // one-shot response to this key.
        }

        // Only the ARM step - sets the flag and LED. The actual continuous
        // TWR/altitude/velocity monitor and auto-fire-at-trigger-point logic
        // (SUICIDEBURN_IMMINENT / SUICIDEBURN_BURNING) is real flight-
        // dynamics math (estimating stopping distance from current
        // velocity/TWR/altitude) - that's part of the "hard" batch, not
        // implemented here. suicideBurnArmed exists so that monitor has
        // something to check once it's written.
        void DoSuicideBurnArm()
        {
            suicideBurnArmed = true;
            SetLEDState(0x0A, LEDStates.SUICIDEBURN_ARMED, 0x00);
        }

        // UNVERIFIED: exact IScienceDataContainer/IScienceDataTransmitter
        // API shape (method names, whether DumpData is the right call here)
        // needs checking against the real KSP assemblies.
        void DoTransmitScience()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            List<IScienceDataTransmitter> transmitters = vessel.FindPartModulesImplementing<IScienceDataTransmitter>();
            if (transmitters.Count == 0)
            {
                SetLEDState(0x0C, LEDStates.SCIENCE_UNAVAILABLE, 0x00);
                return;
            }

            bool anyData = false;
            foreach (var container in vessel.FindPartModulesImplementing<IScienceDataContainer>())
            {
                ScienceData[] data = container.GetData();
                if (data.Length == 0)
                    continue;

                anyData = true;
                foreach (var d in data)
                    transmitters[0].TransmitData(new List<ScienceData> { d });
            }

            // SCIENCE_COMPLETE isn't set here - transmission finishes
            // asynchronously in-game, which would need a completion
            // callback/event, not something known at the moment of the
            // keypress.
            SetLEDState(0x0C, anyData ? LEDStates.SCIENCE_TRANSMITTING : LEDStates.SCIENCE_UNAVAILABLE, 0x00);
        }

        // No real companion-app IPC exists yet (tech stack still undecided
        // per the project doc), so this only flips a local flag - nothing
        // actually opens/closes on screen until that channel is built.
        // RESOURCE MONITOR's own LED is already driven continuously by
        // CheckLEDStates from real resource levels, independent of this.
        void DoResourceMonitorToggle()
        {
            resourceMonitorPanelOpen = !resourceMonitorPanelOpen;
            Debug.Log("[KSPMacropad] Resource monitor panel " +
                (resourceMonitorPanelOpen ? "OPEN (not yet wired to companion app)" : "CLOSED"));
        }

    }


}