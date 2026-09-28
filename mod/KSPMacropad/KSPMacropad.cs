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
        // states exist for them.
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
        private byte[] ledData = new byte[TrackedKeyIds.Length + 1];

        private bool running = false;

        // Reserved inbound-to-pad IDs outside the key (0x00-0x0F) / underglow
        // (0x10-0x13) ranges - same 5-byte frame as UpdateLED, just new
        // meanings. Must match firmware's HEARTBEAT_ID/THROTTLE_TELEMETRY_ID/
        // WARP_TELEMETRY_ID/*_MACRO_ID/QUEUE_LENGTH_ID exactly.
        private const byte HEARTBEAT_ID = 0x14;
        private const byte THROTTLE_TELEMETRY_ID = 0x15;
        private const byte WARP_TELEMETRY_ID = 0x16;
        private const byte ACTIVE_MACRO_TELEMETRY_ID = 0x17; // data = key id of the flying macro, NO_MACRO if none
        private const byte NEXT_MACRO_TELEMETRY_ID = 0x18;   // data = key id at the head of the queue, NO_MACRO if empty
        private const byte QUEUE_LENGTH_TELEMETRY_ID = 0x19; // data = number of queued macros (not counting the active one)

        private const float HEARTBEAT_INTERVAL = 1.0f; // seconds; must stay well under firmware's HEARTBEAT_TIMEOUT (3.0s)

        private float lastHeartbeatTime = 0f;
        private int lastSentThrottle = -1; // -1 = never sent yet, forces the first send
        private int lastSentWarp = -1;
        private int lastSentActiveMacro = -1;
        private int lastSentNextMacro = -1;
        private int lastSentQueueLength = -1;

        private bool suicideBurnArmed = false;
        private bool resourceMonitorPanelOpen = false;

        // AUTOPILOT state machine (see StartAutopilot/TickAutopilot below).
        private enum AutopilotPhase
        {
            Idle,
            WaitEjectionBurn,
            BurningEjection,
            WaitMidCourse,
            BurningMidCourse,
            WaitDestinationSOI,
            WaitCaptureBurn,
            BurningCapture
        }
        private AutopilotPhase autopilotPhase = AutopilotPhase.Idle;
        private CelestialBody autopilotDestination;
        private ManeuverNode autopilotActiveNode;
        private double autopilotMidCourseUT;
        private double autopilotTransferTime;
        private double autopilotDeadlineUT; // abort if we blow past this without reaching the destination SOI
        private const double AUTOPILOT_DV_EPSILON = 0.1; // m/s - burn considered "done" below this

        // Flying-macro queue (see RequestFlyingMacro). Only one macro flies
        // the vessel at a time; key presses for others while busy wait here.
        private enum MacroStart { Started, Done, Failed }
        private const byte NO_MACRO = 0xFF;
        private byte activeMacroKey = NO_MACRO;
        private readonly List<byte> macroQueue = new List<byte>();

        // Shared single-node burn executor used by CIRCULARIZE, DEORBIT,
        // INTERCEPT and ORBIT SYNC (see StartNodeJob/TickNodeJob).
        private enum NodeJobPhase { None, Warping, Burning }
        private NodeJobPhase nodeJobPhase = NodeJobPhase.None;
        private ManeuverNode nodeJobNode;
        private byte nodeJobKeyId;
        private byte nodeJobIdleState;
        private byte nodeJobBurningState;
        private Action nodeJobOnComplete;

        // Burn-completion tracking for ExecuteNodeBurn (one burn at a time).
        private ManeuverNode activeBurnNode;
        private Vector3d activeBurnStartDir;

        private const double NODE_WARP_LEAD_SECONDS = 30.0;  // stop warping this long before a node so SAS can align
        private const double NODE_ALIGN_TOLERANCE_DEG = 5.0; // no throttle until pointed within this of the burn vector
        private const double NODE_TAPER_DV = 10.0;           // m/s remaining below which throttle scales down
        private const double NODE_MIN_LEAD_SECONDS = 60.0;   // never plan a node closer than this to now

        private ManeuverNode interceptPlannedNode;

        private const double DEORBIT_ATMOSPHERE_FRACTION = 0.5; // target periapsis at this fraction of atmosphere depth

        private const int INTERCEPT_DEPARTURE_SAMPLES = 60;
        private const int INTERCEPT_TOF_SAMPLES = 15;
        private const double INTERCEPT_TOF_MIN_FRACTION = 0.3;
        private const double INTERCEPT_TOF_MAX_FRACTION = 1.5;
        private const double INTERCEPT_MAX_WINDOW_ORBITS = 5.0;
        private const double INTERCEPT_SAFE_ALTITUDE_MARGIN = 10000.0; // m above surface/atmosphere a transfer may dip to

        private const double ORBITSYNC_MIN_REL_INCLINATION_DEG = 0.05;
        private const int ORBITSYNC_CROSSING_SAMPLES = 180;

        private bool rendezvousActive = false;
        private bool rendezvousPrevPrecisionMode = false;
        private const double RENDEZVOUS_IN_RANGE_METERS = 200.0;  // placeholder
        private const double RENDEZVOUS_MAX_CLOSING_MS = 50.0;    // closing speed that maps to data byte 255

        // Encoder state. encoderMode and the dial targets mirror the
        // firmware's own copies (same toggles, same clamping), so a value
        // committed here is the value the pad's OLED was showing.
        private int encoderMode = 1;
        private int precisionThrottleTarget = 0;   // mode 2 left, 0-100 %
        private int precisionHeadingTarget = 0;    // mode 2 right, 0-359 deg
        private int rcsLimiterTarget = 0;          // mode 3 right, 0-100 %
        private bool precisionThrottleDialed = false;
        private bool precisionHeadingDialed = false;
        private bool rcsLimiterDialed = false;

        private bool headingHoldActive = false;
        private double headingHoldPitchDeg;
        private double headingHoldHeadingDeg;

        private const float THROTTLE_STEP_PER_DETENT = 0.02f;  // mode 1 left: 2% per click
        private const double ZOOM_FACTOR_PER_DETENT = 1.1;     // mode 3 left: 10% closer/farther per click
        private const float MANUAL_INPUT_THRESHOLD = 0.1f;     // pitch/yaw/roll input that releases a heading hold

        private bool agtActive = false;
        private const double AGT_START_ALTITUDE = 10000.0;
        private const double AGT_END_ALTITUDE = 45000.0;
        private const double AGT_START_PITCH_DEG = 90.0;
        private const double AGT_END_PITCH_DEG = 45.0;
        private const double AGT_TARGET_APOAPSIS = 80000.0; // placeholder - cut throttle and hand off to CIRCULARIZE here
        private const double AGT_MAX_Q_KPA = 20.0;          // placeholder - above this, stay close to surface prograde
        private const double AGT_MAX_AOA_DEG = 5.0;
        private const double AGT_MIN_SRF_SPEED_FOR_AOA = 50.0; // surface prograde is too noisy to follow below this

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
                                RequestFlyingMacro(0x01);
                                break;

                            case 0x02:
                                Debug.Log("[KSPMacropad] KEY PRESSED: CIRCULARIZE");
                                RequestFlyingMacro(0x02);
                                break;

                            case 0x03:
                                Debug.Log("[KSPMacropad] KEY PRESSED: TIME ACCELERATION TO NXT BURN");
                                DoTimeAccelToNextEvent();
                                break;

                            case 0x04:
                                Debug.Log("[KSPMacropad] KEY PRESSED: INTERCEPT CALCULATION");
                                RequestFlyingMacro(0x04);
                                break;

                            case 0x05:
                                Debug.Log("[KSPMacropad] KEY PRESSED: ORBIT SYNC");
                                RequestFlyingMacro(0x05);
                                break;

                            case 0x06:
                                Debug.Log("[KSPMacropad] KEY PRESSED: RENDEZVOUS PREPARATION");
                                DoRendezvousPrep();
                                break;

                            case 0x07:
                                Debug.Log("[KSPMacropad] KEY PRESSED: DEORBIT BURN");
                                RequestFlyingMacro(0x07);
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
                                OnPrecisionInputKey();
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
                                OnAuxModeKey();
                                break;

                            case 0x0F:
                                Debug.Log("[KSPMacropad] KEY PRESSED: AUTOPILOT");
                                RequestFlyingMacro(0x0F);
                                break;
                        }

                        break;

                    case 0x02:
                        // id = (mode << 4) | encoder, encoder 1 = left, 2 = right
                        int encoderModeNibble = msg[2] >> 4;
                        int encoderSide = msg[2] & 0x0F;
                        short steps = (short)((msg[3] << 8) | msg[4]);
                        Debug.Log("[KSPMacropad] TURNED ENCODER(mode " + encoderModeNibble + "): " +
                            (encoderSide == 1 ? "LEFT" : "RIGHT") + " " + steps);
                        HandleEncoder(encoderModeNibble, encoderSide, steps);
                        break;
                }

            }

            SendHeartbeat();
            SendTelemetry();
            SendMacroQueueTelemetry();
            CheckLEDStates();
            TickAutopilot();
            TickNodeJob();
            TickGravityTurn();
            TickRendezvous();
            TickHeadingHold();
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

        // Active macro / next queued macro / queue length for the pad's
        // OLED. Same diff-then-send pattern as SendTelemetry, but kept out
        // of it since the queue exists even with no active vessel.
        void SendMacroQueueTelemetry()
        {
            int active = activeMacroKey;
            if (active != lastSentActiveMacro)
            {
                lastSentActiveMacro = active;
                UpdateLED(ACTIVE_MACRO_TELEMETRY_ID, 0x00, (byte)active);
            }

            int next = macroQueue.Count > 0 ? macroQueue[0] : NO_MACRO;
            if (next != lastSentNextMacro)
            {
                lastSentNextMacro = next;
                UpdateLED(NEXT_MACRO_TELEMETRY_ID, 0x00, (byte)next);
            }

            int length = Math.Min(macroQueue.Count, 255);
            if (length != lastSentQueueLength)
            {
                lastSentQueueLength = length;
                UpdateLED(QUEUE_LENGTH_TELEMETRY_ID, 0x00, (byte)length);
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
            Array.Clear(ledData, 0, ledData.Length);

            // Iterates the real tracked-key list (not a raw 0x00-0x0D range),
            // since 0x0B has no LED states and 0x0F (AUTOPILOT) does.
            foreach (byte keyId in TrackedKeyIds)
            {
                UpdateLED(keyId, 0x00, 0x00);
            }

            UpdateUnderglow(0x00, 0x00);
        }

        // Only sends an update when the state or data byte actually differs
        // from what was last sent for that key - same "diff, don't spam"
        // pattern as the rest of the LED protocol. Data is diffed too since
        // RENDEZVOUS PREP streams closing velocity through it.
        void SetLEDState(byte keyId, byte state, byte data)
        {
            int idx = KeyIdToStateIndex[keyId];
            if (ledStates[idx] != state || ledData[idx] != data)
            {
                ledStates[idx] = state;
                ledData[idx] = data;
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

        // ------------------------------------------------------------------
        // AUTOPILOT (0x0F) - full sequence: dv feasibility check, ejection
        // burn, warp, mid-course correction, warp, capture burn, all
        // auto-executed and driven from TickAutopilot() every frame.
        // Destination is auto-picked from whatever the player already has
        // targeted in KSP's own UI (FlightGlobals.fetch.VesselTarget) - no
        // dedicated encoder mode or companion-app picker exists yet (mode 3's
        // encoders are already spoken for: zoom/RCS limiter), so there is
        // currently no other channel for "which body" to reach the mod.
        // Revisit once either a 4th encoder mode or the companion app's
        // destination picker exists.
        //
        // Departure/arrival timing and magnitudes come from
        // FindBestTransferWindow()'s Lambert search below.
        //
        // Known simplifying assumptions (not tuned/verified in-game):
        //   - the ejection burn is prograde only. The hyperbolic-excess
        //     vector the Lambert solve produces generally has a radial/
        //     normal component at the burn point; only its magnitude is
        //     used, which leaves residual pointing error.
        //   - no SOI-transit-time offset: the burn happens AT the found
        //     departure UT rather than early enough to be at that velocity
        //     when crossing the SOI boundary.
        //   - the search grid (vessel's own periapsis passages x a handful
        //     of time-of-flight samples per passage) is coarse - a
        //     reasonable window, not the cheapest possible one.
        //   - mid-course correction only matches the destination's orbital
        //     plane, not a full aim-point correction for the two points
        //     above.
        //   - the feasibility check budgets capture into a +100km parking
        //     orbit; the actual capture burn circularizes at whatever
        //     periapsis the vessel arrives with.
        //   - no Principia (n-body) detection - doc already flags AUTOPILOT
        //     as broken under Principia; not checked for here.
        //   - Coordinate frame for FindBestTransferWindow's vector math:
        //     the KSP API docs state "all Vector3d's returned by Orbit class
        //     functions have their y and z axes flipped", so position and
        //     velocity vectors read from Orbit methods share one frame and
        //     can be combined directly. Documented for the class as a whole,
        //     not per-method - recheck once this runs against KSP.
        MacroStart StartAutopilot()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target == null)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: no target selected, nothing to plan against");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return MacroStart.Failed;
            }

            CelestialBody destinationBody = target as CelestialBody;
            if (destinationBody == null && target is Vessel targetVessel)
                destinationBody = targetVessel.mainBody;

            if (destinationBody == null)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: target has no resolvable body");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return MacroStart.Failed;
            }

            CelestialBody originBody = vessel.mainBody;

            if (destinationBody == originBody)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: target is in the same SOI, not an interplanetary case");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return MacroStart.Failed;
            }

            // MVP only handles "vessel orbiting a planet, transferring to
            // another planet" - nested-moon-to-elsewhere cases (or a vessel
            // already heliocentric) aren't covered yet.
            if (originBody.referenceBody == null || originBody.referenceBody != Planetarium.fetch.Sun)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: origin body's parent isn't the Sun - unsupported case for this MVP");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return MacroStart.Failed;
            }

            bool foundWindow = FindBestTransferWindow(vessel, originBody, destinationBody,
                out double departureUT, out double arrivalUT, out double ejectionDv, out double captureDv);

            if (!foundWindow)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: no valid transfer window found in the search window");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IMPOSSIBLE, 0x00);
                return MacroStart.Failed;
            }

            double requiredDv = ejectionDv + captureDv;
            double availableDv = GetVesselDeltaV(vessel);

            Debug.Log("[KSPMacropad] AUTOPILOT: departureUT=" + departureUT + " arrivalUT=" + arrivalUT +
                " required dv=" + requiredDv + " (ejection=" + ejectionDv + " capture=" + captureDv +
                ") available dv=" + availableDv);

            if (availableDv < requiredDv)
            {
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IMPOSSIBLE, 0x00);
                return MacroStart.Failed;
            }

            // Sufficient dv and a real departure window - kick off the
            // sequence. Ejection burn is planned at the found departure UT
            // (one of the vessel's own periapsis passages - see
            // FindBestTransferWindow), sized to the solved ejection dv,
            // pointed prograde (see the pointing-error caveat above).
            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(departureUT);
            node.DeltaV = new Vector3d(0, 0, ejectionDv); // confirmed real field: (radial-plus, normal-minus, prograde)
            vessel.patchedConicSolver.UpdateFlightPlan(); // confirmed real method, no documented params

            autopilotDestination = destinationBody;
            autopilotTransferTime = arrivalUT - departureUT;
            autopilotActiveNode = node;
            autopilotDeadlineUT = arrivalUT + Math.Max(autopilotTransferTime * 0.25, 6.0 * 3600.0); // solved arrival time plus slack, not a re-guessed one
            autopilotPhase = AutopilotPhase.WaitEjectionBurn;

            SetLEDState(0x0F, LEDStates.AUTOPILOT_PLANNING, 0x00);
            WarpToBurn(departureUT);
            return MacroStart.Started;
        }

        // Drives the AUTOPILOT state machine forward one tick. No-op when
        // idle. Every phase transition logs, same as the rest of this file.
        void TickAutopilot()
        {
            if (autopilotPhase == AutopilotPhase.Idle)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            double currentUT = Planetarium.GetUniversalTime();

            if (currentUT > autopilotDeadlineUT)
            {
                AbortAutopilot(vessel, "exceeded expected transfer time without reaching the destination SOI " +
                    "(ejection burn pointing is prograde-only - see caveats on StartAutopilot)");
                return;
            }

            switch (autopilotPhase)
            {
                case AutopilotPhase.WaitEjectionBurn:
                    if (currentUT >= autopilotActiveNode.UT)
                        autopilotPhase = AutopilotPhase.BurningEjection;
                    break;

                case AutopilotPhase.BurningEjection:
                    if (ExecuteNodeBurn(vessel, autopilotActiveNode))
                    {
                        Debug.Log("[KSPMacropad] AUTOPILOT: ejection burn complete");
                        autopilotActiveNode = null;
                        autopilotMidCourseUT = currentUT + autopilotTransferTime / 2.0;
                        autopilotPhase = AutopilotPhase.WaitMidCourse;
                        SetLEDState(0x0F, LEDStates.AUTOPILOT_EXECUTING, 0x00);
                        WarpToBurn(autopilotMidCourseUT);
                    }
                    break;

                case AutopilotPhase.WaitMidCourse:
                    if (currentUT >= autopilotMidCourseUT)
                    {
                        ManeuverNode correctionNode = BuildMidCourseCorrectionNode(vessel, autopilotDestination, currentUT);
                        if (correctionNode != null)
                        {
                            autopilotActiveNode = correctionNode;
                            autopilotPhase = AutopilotPhase.BurningMidCourse;
                        }
                        else
                        {
                            Debug.Log("[KSPMacropad] AUTOPILOT: relative inclination negligible, skipping mid-course correction");
                            autopilotPhase = AutopilotPhase.WaitDestinationSOI;
                        }
                    }
                    break;

                case AutopilotPhase.BurningMidCourse:
                    if (ExecuteNodeBurn(vessel, autopilotActiveNode))
                    {
                        Debug.Log("[KSPMacropad] AUTOPILOT: mid-course correction complete");
                        autopilotActiveNode = null;
                        autopilotPhase = AutopilotPhase.WaitDestinationSOI;
                    }
                    break;

                case AutopilotPhase.WaitDestinationSOI:
                    // Polls the game's own patched-conic SOI tracking rather
                    // than assuming our burn worked - see the big caveat
                    // above. Nothing to warp to here since we don't actually
                    // know when (or if) this will happen.
                    if (vessel.mainBody == autopilotDestination)
                    {
                        Debug.Log("[KSPMacropad] AUTOPILOT: entered destination SOI, planning capture burn");
                        // Circularize at the arrival periapsis, whatever
                        // altitude that ended up being.
                        double periapsisUT = currentUT + vessel.orbit.timeToPe;
                        ManeuverNode captureNode = vessel.patchedConicSolver.AddManeuverNode(periapsisUT);
                        SetNodeFromOrbitFrameDv(vessel, captureNode, CircularizeDvAt(vessel.orbit, autopilotDestination.gravParameter, periapsisUT));

                        autopilotActiveNode = captureNode;
                        autopilotPhase = AutopilotPhase.WaitCaptureBurn;
                        WarpToBurn(periapsisUT);
                    }
                    break;

                case AutopilotPhase.WaitCaptureBurn:
                    if (currentUT >= autopilotActiveNode.UT)
                        autopilotPhase = AutopilotPhase.BurningCapture;
                    break;

                case AutopilotPhase.BurningCapture:
                    if (ExecuteNodeBurn(vessel, autopilotActiveNode))
                    {
                        Debug.Log("[KSPMacropad] AUTOPILOT: capture burn complete, transfer finished");
                        autopilotActiveNode = null;
                        autopilotDestination = null;
                        autopilotPhase = AutopilotPhase.Idle;
                        SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                        OnMacroFinished(0x0F, true);
                    }
                    break;
            }
        }

        // Mid-course correction: rotates the vessel's velocity into the
        // destination body's orbital plane, keeping its magnitude. Built as
        // a full vector (PlaneMatchDv), so the burn direction falls out of
        // the geometry rather than needing an ascending/descending-node
        // sign decision. Returns null if the planes already match.
        ManeuverNode BuildMidCourseCorrectionNode(Vessel vessel, CelestialBody destination, double ut)
        {
            Vector3d destNormal = OrbitNormalAt(destination.orbit, ut);
            Vector3d vesselNormal = OrbitNormalAt(vessel.orbit, ut);
            if (OrbitMath.AngleDeg(vesselNormal, destNormal) < ORBITSYNC_MIN_REL_INCLINATION_DEG)
                return null;

            Vector3d dv = OrbitMath.PlaneMatchDv(vessel.orbit.getOrbitalVelocityAtUT(ut), destNormal);
            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(ut);
            SetNodeFromOrbitFrameDv(vessel, node, dv);
            return node;
        }

        // Bails out of an in-progress AUTOPILOT sequence: kills throttle,
        // clears any pending node, resets the state machine to Idle and the
        // LED back to AUTOPILOT_IDLE (not IMPOSSIBLE - the dv check already
        // passed, this is a targeting miss, not a dv shortfall).
        void AbortAutopilot(Vessel vessel, string reason)
        {
            Debug.Log("[KSPMacropad] AUTOPILOT: aborting - " + reason);

            FlightInputHandler.state.mainThrottle = 0f;
            if (vessel != null)
                vessel.Autopilot.Enabled = false;

            if (autopilotActiveNode != null && vessel != null && vessel.patchedConicSolver != null)
                vessel.patchedConicSolver.RemoveManeuverNode(autopilotActiveNode);

            autopilotActiveNode = null;
            autopilotDestination = null;
            autopilotPhase = AutopilotPhase.Idle;
            SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
            OnMacroFinished(0x0F, false);
        }

        // ------------------------------------------------------------------
        // Transfer-window search. Solves for WHEN to leave, not just how big
        // a burn - a correctly sized Hohmann burn at the wrong time reaches
        // the right radius with the destination somewhere else.
        //
        // The Lambert solver lives in OrbitMath (bottom of this file) and is
        // compiled and unit-tested outside KSP against a published textbook
        // case (Curtis, "Orbital Mechanics for Engineering Students",
        // Example 5.2). The KSP-facing code around it has not been run in
        // game yet.
        // ------------------------------------------------------------------

        private const int TRANSFER_SEARCH_MAX_DEPARTURES = 60;
        private const int TRANSFER_SEARCH_TOF_SAMPLES = 8;
        private const double TRANSFER_SEARCH_TOF_MIN_FRACTION = 0.5;
        private const double TRANSFER_SEARCH_TOF_MAX_FRACTION = 1.5;

        // Searches the vessel's own upcoming periapsis passages (where the
        // ejection burn happens) crossed with a spread of times-of-flight
        // around the analytic Hohmann estimate, Lambert-solving each pair
        // and keeping the one with the lowest ejection+capture dv. Returns
        // false if nothing in the grid produced a valid Lambert solution.
        bool FindBestTransferWindow(Vessel vessel, CelestialBody originBody, CelestialBody destinationBody,
            out double bestDepartureUT, out double bestArrivalUT, out double bestEjectionDv, out double bestCaptureDv)
        {
            bestDepartureUT = 0;
            bestArrivalUT = 0;
            bestEjectionDv = 0;
            bestCaptureDv = 0;
            bool found = false;
            double bestTotalDv = double.MaxValue;

            double muSun = Planetarium.fetch.Sun.gravParameter;

            double aTransferApprox = (originBody.orbit.semiMajorAxis + destinationBody.orbit.semiMajorAxis) / 2.0;
            double hohmannEstimate = Math.PI * Math.Sqrt(Math.Pow(aTransferApprox, 3) / muSun);

            double synodicPeriod = OrbitMath.SynodicPeriod(originBody.orbit.period, destinationBody.orbit.period);

            double currentUT = Planetarium.GetUniversalTime();
            double vesselPeriod = vessel.orbit.period;
            bool vesselPeriodValid = vesselPeriod > 0 && !double.IsNaN(vesselPeriod);

            int maxCandidates = vesselPeriodValid
                ? Math.Min(TRANSFER_SEARCH_MAX_DEPARTURES, (int)Math.Ceiling(synodicPeriod / vesselPeriod) + 1)
                : 1; // not on a stable elliptical orbit - only the immediate next periapsis is usable

            double departureUT = currentUT + vessel.orbit.timeToPe;

            for (int d = 0; d < maxCandidates && departureUT < currentUT + synodicPeriod; d++)
            {
                Vector3d r1vec = originBody.orbit.getRelativePositionAtUT(departureUT);
                Vector3d vOriginAtDep = originBody.orbit.getOrbitalVelocityAtUT(departureUT);
                Vector3d refNormal = Vector3d.Cross(r1vec, vOriginAtDep);

                for (int t = 0; t < TRANSFER_SEARCH_TOF_SAMPLES; t++)
                {
                    double frac = TRANSFER_SEARCH_TOF_MIN_FRACTION +
                        (TRANSFER_SEARCH_TOF_MAX_FRACTION - TRANSFER_SEARCH_TOF_MIN_FRACTION) * t / (TRANSFER_SEARCH_TOF_SAMPLES - 1);
                    double tof = hohmannEstimate * frac;
                    double arrivalUT = departureUT + tof;

                    Vector3d r2vec = destinationBody.orbit.getRelativePositionAtUT(arrivalUT);
                    Vector3d vDestAtArr = destinationBody.orbit.getOrbitalVelocityAtUT(arrivalUT);

                    if (!OrbitMath.SolveLambert(r1vec, r2vec, tof, muSun, refNormal, out Vector3d vTransferAtDep, out Vector3d vTransferAtArr))
                        continue;

                    double vInfDepart = (vTransferAtDep - vOriginAtDep).magnitude;
                    double vInfArrive = (vDestAtArr - vTransferAtArr).magnitude;

                    double ejectionDv = VInfToEjectionDv(vessel, originBody, vInfDepart);
                    double captureDv = VInfToCaptureDv(destinationBody, vInfArrive);
                    double totalDv = ejectionDv + captureDv;

                    if (totalDv < bestTotalDv)
                    {
                        bestTotalDv = totalDv;
                        bestDepartureUT = departureUT;
                        bestArrivalUT = arrivalUT;
                        bestEjectionDv = ejectionDv;
                        bestCaptureDv = captureDv;
                        found = true;
                    }
                }

                departureUT += vesselPeriodValid ? vesselPeriod : synodicPeriod;
            }

            return found;
        }

        // dv from a circular parking orbit around originBody up to
        // hyperbolic excess speed vInf (magnitude only - see the
        // pointing-error caveat on StartAutopilot).
        double VInfToEjectionDv(Vessel vessel, CelestialBody originBody, double vInf)
        {
            double muOrigin = originBody.gravParameter;
            double rPark = vessel.orbit.semiMajorAxis;
            double vHyperbolicAtPark = Math.Sqrt(vInf * vInf + 2.0 * muOrigin / rPark);
            return vHyperbolicAtPark - OrbitMath.CircularSpeed(muOrigin, rPark);
        }

        // dv from hyperbolic arrival speed vInf down into a circular
        // parking orbit at destinationBody.Radius + 100km (placeholder
        // altitude - not chosen by the player yet).
        double VInfToCaptureDv(CelestialBody destinationBody, double vInf)
        {
            double muDest = destinationBody.gravParameter;
            double rCapture = destinationBody.Radius + 100000.0;
            double vHyperbolicAtCapture = Math.Sqrt(vInf * vInf + 2.0 * muDest / rCapture);
            return vHyperbolicAtCapture - OrbitMath.CircularSpeed(muDest, rCapture);
        }

        // ------------------------------------------------------------------
        // Shared maneuver-node plumbing: building a node from a desired dv
        // vector, warping to it, and flying the burn. Used by CIRCULARIZE,
        // DEORBIT, INTERCEPT, ORBIT SYNC and AUTOPILOT.
        // ------------------------------------------------------------------

        bool IsFlightComputerBusy()
        {
            return activeMacroKey != NO_MACRO;
        }

        // ------------------------------------------------------------------
        // Flying-macro queue. AGT, CIRCULARIZE, INTERCEPT, ORBIT SYNC,
        // DEORBIT and AUTOPILOT all take over throttle and SAS, so only one
        // runs at a time. Pressing one of those keys:
        //   - while it's the one running: cancels it
        //   - while it's already queued: removes it from the queue
        //   - while another is running: adds it to the end of the queue
        //   - otherwise: starts it now
        // Queued macros are planned when they start, not when queued, so
        // each one works from the orbit the previous one actually left.
        // When the running macro succeeds the next one starts; when it
        // fails or is cancelled the whole queue is cleared, so a chain
        // never continues past a step that didn't happen.
        // ------------------------------------------------------------------
        void RequestFlyingMacro(byte keyId)
        {
            if (activeMacroKey == keyId)
            {
                CancelActiveMacro();
                return;
            }

            if (macroQueue.Remove(keyId))
            {
                Debug.Log("[KSPMacropad] QUEUE: removed " + MacroName(keyId));
                return;
            }

            if (IsFlightComputerBusy())
            {
                macroQueue.Add(keyId);
                Debug.Log("[KSPMacropad] QUEUE: " + MacroName(keyId) + " queued behind " + MacroName(activeMacroKey) +
                    " (position " + macroQueue.Count + ")");
                return;
            }

            StartFlyingMacro(keyId, false);
        }

        MacroStart StartFlyingMacro(byte keyId, bool fromQueue)
        {
            // Any other macro changes the orbit a planned-but-unflown
            // intercept was computed from, so that plan is dropped.
            if (keyId != 0x04)
                DiscardPlannedIntercept();

            MacroStart result;
            switch (keyId)
            {
                case 0x01: result = StartAutoGravityTurn(); break;
                case 0x02: result = StartCircularize(); break;
                case 0x04: result = StartInterceptCalc(fromQueue); break;
                case 0x05: result = StartOrbitSync(); break;
                case 0x07: result = StartDeorbitBurn(); break;
                case 0x0F: result = StartAutopilot(); break;
                default: result = MacroStart.Failed; break;
            }

            if (result == MacroStart.Started)
                activeMacroKey = keyId;

            return result;
        }

        // Called exactly once by each flying macro when it ends.
        void OnMacroFinished(byte keyId, bool success)
        {
            if (activeMacroKey != keyId)
                return;

            activeMacroKey = NO_MACRO;

            if (!success)
            {
                if (macroQueue.Count > 0)
                {
                    Debug.Log("[KSPMacropad] QUEUE: " + MacroName(keyId) + " didn't finish, clearing " +
                        macroQueue.Count + " queued macro(s)");
                    macroQueue.Clear();
                }
                return;
            }

            while (macroQueue.Count > 0)
            {
                byte next = macroQueue[0];
                macroQueue.RemoveAt(0);
                Debug.Log("[KSPMacropad] QUEUE: starting " + MacroName(next));

                MacroStart result = StartFlyingMacro(next, true);
                if (result == MacroStart.Started)
                    return;

                if (result == MacroStart.Failed)
                {
                    Debug.Log("[KSPMacropad] QUEUE: " + MacroName(next) + " couldn't start, clearing the rest of the queue");
                    macroQueue.Clear();
                    return;
                }
                // Done: nothing needed flying (e.g. orbit already synced) - move on.
            }
        }

        void CancelActiveMacro()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            switch (activeMacroKey)
            {
                case 0x01: StopGravityTurn(vessel, "cancelled by keypress", false); break;
                case 0x0F: AbortAutopilot(vessel, "cancelled by keypress"); break;
                default: CancelNodeJob("cancelled by keypress"); break;
            }
        }

        static string MacroName(byte keyId)
        {
            switch (keyId)
            {
                case 0x01: return "AUTO GRAVITY TURN";
                case 0x02: return "CIRCULARIZE";
                case 0x04: return "INTERCEPT";
                case 0x05: return "ORBIT SYNC";
                case 0x07: return "DEORBIT";
                case 0x0F: return "AUTOPILOT";
                default: return "0x" + keyId.ToString("X2");
            }
        }

        void DiscardPlannedIntercept()
        {
            if (interceptPlannedNode == null)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel != null && vessel.patchedConicSolver.maneuverNodes.Contains(interceptPlannedNode))
                vessel.patchedConicSolver.RemoveManeuverNode(interceptPlannedNode);

            interceptPlannedNode = null;
            SetLEDState(0x04, LEDStates.INTERCEPT_IDLE, 0x00);
        }

        static Vector3d ToVector3d(Vector3 v)
        {
            return new Vector3d(v.x, v.y, v.z);
        }

        static void EnsureSAS(Vessel vessel)
        {
            if (!vessel.ActionGroups[KSPActionGroup.SAS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.SAS);
        }

        static Vector3d OrbitNormalAt(Orbit orbit, double ut)
        {
            return Vector3d.Cross(orbit.getRelativePositionAtUT(ut), orbit.getOrbitalVelocityAtUT(ut)).normalized;
        }

        // dv (Orbit frame) that turns the velocity at ut into a circular
        // orbit at the current radius, removing any radial component too.
        static Vector3d CircularizeDvAt(Orbit orbit, double mu, double ut)
        {
            Vector3d pos = orbit.getRelativePositionAtUT(ut);
            Vector3d vel = orbit.getOrbitalVelocityAtUT(ut);
            return OrbitMath.Horizontal(pos, vel) * OrbitMath.CircularSpeed(mu, pos.magnitude) - vel;
        }

        // Stops warping NODE_WARP_LEAD_SECONDS before a burn so SAS has time
        // to swing around before the throttle opens.
        void WarpToBurn(double burnUT)
        {
            double target = burnUT - NODE_WARP_LEAD_SECONDS;
            if (target > Planetarium.GetUniversalTime() + 1.0)
                TimeWarp.fetch.WarpTo(target);
        }

        // Sets node.DeltaV from a dv vector in the Orbit frame. Radial and
        // prograde components are dot products, which don't depend on the
        // frame's handedness. The normal component does: ManeuverNode.DeltaV
        // is documented as (radial-plus, normal-MINUS, prograde), and the
        // Orbit frame is y/z-swapped relative to world space, so the sign
        // of the normal axis is resolved empirically instead of assumed -
        // both signs are tried and the one whose resulting
        // GetBurnVector() matches the requested dv is kept.
        void SetNodeFromOrbitFrameDv(Vessel vessel, ManeuverNode node, Vector3d dvOrbitFrame)
        {
            Orbit patch = node.patch ?? vessel.orbit;
            Vector3d pos = patch.getRelativePositionAtUT(node.UT);
            Vector3d vel = patch.getOrbitalVelocityAtUT(node.UT);
            OrbitMath.DecomposeBurn(pos, vel, dvOrbitFrame, out double radial, out double normal, out double prograde);

            Vector3d best = new Vector3d(radial, normal, prograde);
            double bestErr = double.MaxValue;
            for (int sign = 1; sign >= -1; sign -= 2)
            {
                Vector3d candidate = new Vector3d(radial, sign * normal, prograde);
                node.DeltaV = candidate;
                vessel.patchedConicSolver.UpdateFlightPlan();
                Vector3d burn = node.GetBurnVector(patch);
                double err = Math.Min((burn - dvOrbitFrame).magnitude, (burn - OrbitMath.SwapYZ(dvOrbitFrame)).magnitude);
                if (err < bestErr)
                {
                    bestErr = err;
                    best = candidate;
                }
            }

            node.DeltaV = best;
            vessel.patchedConicSolver.UpdateFlightPlan();

            if (bestErr > 0.01 * dvOrbitFrame.magnitude + 0.1)
                Debug.Log("[KSPMacropad] WARNING: node burn vector differs from requested dv by " + bestErr +
                    " m/s - GetBurnVector's frame may not be what SetNodeFromOrbitFrameDv assumes");
        }

        // Flies one node: holds SAS on the node, keeps the throttle closed
        // until pointed within NODE_ALIGN_TOLERANCE_DEG, tapers the throttle
        // over the last NODE_TAPER_DV m/s, and reports done once the
        // remaining dv is under AUTOPILOT_DV_EPSILON or the burn vector has
        // flipped past 90 degrees from where it started (overshoot).
        // Removes the node when done.
        bool ExecuteNodeBurn(Vessel vessel, ManeuverNode node)
        {
            Vector3d burn = node.GetBurnVector(vessel.orbit);
            double remaining = burn.magnitude;

            if (node != activeBurnNode)
            {
                activeBurnNode = node;
                activeBurnStartDir = burn.normalized;
            }

            bool overshot = Vector3d.Dot(burn, activeBurnStartDir) < 0;
            if (remaining < AUTOPILOT_DV_EPSILON || overshot)
            {
                FlightInputHandler.state.mainThrottle = 0f;
                vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
                vessel.patchedConicSolver.RemoveManeuverNode(node);
                activeBurnNode = null;
                return true;
            }

            EnsureSAS(vessel);
            if (vessel.Autopilot.Mode != VesselAutopilot.AutopilotMode.Maneuver)
                vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Maneuver);

            double pointingError = OrbitMath.AngleDeg(ToVector3d(vessel.ReferenceTransform.up), burn);
            float throttle = 0f;
            if (pointingError <= NODE_ALIGN_TOLERANCE_DEG)
                throttle = (float)Math.Max(0.05, Math.Min(1.0, remaining / NODE_TAPER_DV));

            FlightInputHandler.state.mainThrottle = throttle;
            return false;
        }

        void StartNodeJob(Vessel vessel, ManeuverNode node, byte keyId, byte idleState, byte warpingState,
            byte burningState, Action onComplete)
        {
            nodeJobNode = node;
            nodeJobKeyId = keyId;
            nodeJobIdleState = idleState;
            nodeJobBurningState = burningState;
            nodeJobOnComplete = onComplete;
            nodeJobPhase = NodeJobPhase.Warping;

            SetLEDState(keyId, warpingState, 0x00);
            WarpToBurn(node.UT);
        }

        void TickNodeJob()
        {
            if (nodeJobPhase == NodeJobPhase.None)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (!vessel.patchedConicSolver.maneuverNodes.Contains(nodeJobNode))
            {
                CancelNodeJob("node was deleted");
                return;
            }

            double currentUT = Planetarium.GetUniversalTime();

            if (nodeJobPhase == NodeJobPhase.Warping)
            {
                if (currentUT >= nodeJobNode.UT)
                {
                    nodeJobPhase = NodeJobPhase.Burning;
                    SetLEDState(nodeJobKeyId, nodeJobBurningState, 0x00);
                }
                else if (currentUT >= nodeJobNode.UT - NODE_WARP_LEAD_SECONDS && TimeWarp.CurrentRateIndex == 0)
                {
                    EnsureSAS(vessel);
                    if (vessel.Autopilot.Mode != VesselAutopilot.AutopilotMode.Maneuver)
                        vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Maneuver);
                }
            }

            if (nodeJobPhase == NodeJobPhase.Burning && ExecuteNodeBurn(vessel, nodeJobNode))
            {
                Action onComplete = nodeJobOnComplete;
                nodeJobPhase = NodeJobPhase.None;
                nodeJobNode = null;
                nodeJobOnComplete = null;
                onComplete?.Invoke();
            }
        }

        void CancelNodeJob(string reason)
        {
            Debug.Log("[KSPMacropad] Node burn cancelled - " + reason);
            FlightInputHandler.state.mainThrottle = 0f;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel != null && nodeJobNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(nodeJobNode))
                vessel.patchedConicSolver.RemoveManeuverNode(nodeJobNode);

            byte cancelledKey = nodeJobKeyId;
            SetLEDState(cancelledKey, nodeJobIdleState, 0x00);
            nodeJobPhase = NodeJobPhase.None;
            nodeJobNode = null;
            nodeJobOnComplete = null;
            activeBurnNode = null;
            OnMacroFinished(cancelledKey, false);
        }

        bool HasDeltaVFor(Vessel vessel, double requiredDv, string macroName)
        {
            double availableDv = GetVesselDeltaV(vessel);
            if (availableDv >= requiredDv)
                return true;

            Debug.Log("[KSPMacropad] " + macroName + ": needs " + requiredDv + " m/s, vessel has " + availableDv + " m/s");
            return false;
        }

        // ------------------------------------------------------------------
        // CIRCULARIZE (0x02): circularize at the next apoapsis.
        // ------------------------------------------------------------------
        MacroStart StartCircularize()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            Orbit orbit = vessel.orbit;
            CelestialBody body = vessel.mainBody;
            Vessel.Situations s = vessel.situation;

            if (s == Vessel.Situations.PRELAUNCH || s == Vessel.Situations.LANDED || s == Vessel.Situations.SPLASHED ||
                orbit.eccentricity >= 1.0 || orbit.ApA <= 0)
            {
                Debug.Log("[KSPMacropad] CIRCULARIZE: no apoapsis to circularize at");
                SetLEDState(0x02, LEDStates.CIRC_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            if (body.atmosphere && orbit.ApA < body.atmosphereDepth)
            {
                Debug.Log("[KSPMacropad] CIRCULARIZE: apoapsis is inside the atmosphere");
                SetLEDState(0x02, LEDStates.CIRC_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            SetLEDState(0x02, LEDStates.CIRC_CALCULATING, 0x00);

            double burnUT = Planetarium.GetUniversalTime() + orbit.timeToAp;
            Vector3d dv = CircularizeDvAt(orbit, body.gravParameter, burnUT);

            if (!HasDeltaVFor(vessel, dv.magnitude, "CIRCULARIZE"))
            {
                SetLEDState(0x02, LEDStates.CIRC_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(burnUT);
            SetNodeFromOrbitFrameDv(vessel, node, dv);

            // CIRC has no dedicated burning state - WARPING stays lit through the burn.
            StartNodeJob(vessel, node, 0x02, LEDStates.CIRC_IDLE, LEDStates.CIRC_WARPING, LEDStates.CIRC_WARPING,
                () =>
                {
                    SetLEDState(0x02, LEDStates.CIRC_COMPLETE, 0x00);
                    OnMacroFinished(0x02, true);
                });
            return MacroStart.Started;
        }

        // ------------------------------------------------------------------
        // DEORBIT BURN (0x07): retrograde SAS, then a burn at apoapsis that
        // drops periapsis to DEORBIT_ATMOSPHERE_FRACTION of the atmosphere's
        // depth (or to sea level on an airless body).
        // ------------------------------------------------------------------
        MacroStart StartDeorbitBurn()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            Orbit orbit = vessel.orbit;
            CelestialBody body = vessel.mainBody;

            if (vessel.situation != Vessel.Situations.ORBITING)
            {
                Debug.Log("[KSPMacropad] DEORBIT: not in a stable orbit");
                SetLEDState(0x07, LEDStates.DEORBIT_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            double targetPeR = body.Radius + (body.atmosphere ? body.atmosphereDepth * DEORBIT_ATMOSPHERE_FRACTION : 0.0);
            if (orbit.PeR <= targetPeR)
            {
                Debug.Log("[KSPMacropad] DEORBIT: periapsis is already at or below the deorbit target");
                SetLEDState(0x07, LEDStates.DEORBIT_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            EnsureSAS(vessel);
            vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Retrograde);

            double burnUT = Planetarium.GetUniversalTime() + orbit.timeToAp;
            Vector3d pos = orbit.getRelativePositionAtUT(burnUT);
            Vector3d vel = orbit.getOrbitalVelocityAtUT(burnUT);
            double r = pos.magnitude;
            Vector3d vNew = OrbitMath.Horizontal(pos, vel) * OrbitMath.VisVivaSpeed(body.gravParameter, r, (r + targetPeR) / 2.0);
            Vector3d dv = vNew - vel;

            if (!HasDeltaVFor(vessel, dv.magnitude, "DEORBIT"))
            {
                SetLEDState(0x07, LEDStates.DEORBIT_UNAVAILABLE, 0x00);
                return MacroStart.Failed;
            }

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(burnUT);
            SetNodeFromOrbitFrameDv(vessel, node, dv);
            SetLEDState(0x07, LEDStates.DEORBIT_PLANNED, 0x00);

            StartNodeJob(vessel, node, 0x07, LEDStates.DEORBIT_IDLE, LEDStates.DEORBIT_WARPING, LEDStates.DEORBIT_BURNING,
                () =>
                {
                    SetLEDState(0x07, LEDStates.DEORBIT_IDLE, 0x00);
                    OnMacroFinished(0x07, true);
                });
            return MacroStart.Started;
        }

        // Current target's orbit, only if it orbits the same body as the
        // vessel (INTERCEPT and ORBIT SYNC both work within one SOI).
        Orbit GetTargetOrbitInSameSOI(Vessel vessel, string macroName)
        {
            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target == null)
            {
                Debug.Log("[KSPMacropad] " + macroName + ": no target selected");
                return null;
            }

            Orbit targetOrbit = target.GetOrbit();
            if (targetOrbit == null || targetOrbit.referenceBody != vessel.mainBody)
            {
                Debug.Log("[KSPMacropad] " + macroName + ": target isn't orbiting the same body as the vessel");
                return null;
            }

            if (vessel.orbit.eccentricity >= 1.0)
            {
                Debug.Log("[KSPMacropad] " + macroName + ": vessel isn't in a closed orbit");
                return null;
            }

            return targetOrbit;
        }

        // ------------------------------------------------------------------
        // INTERCEPT CALC (0x04): first press plans the cheapest intercept
        // burn to the current target and shows it (SOLUTION / INSUFFICIENT_DV);
        // second press flies it. When it starts from the queue it plans and
        // flies in one go, since nobody is there to press it twice. LED
        // stays SOLUTION during execution (no executing state is defined for
        // this key), then returns to IDLE.
        //
        // Aims at the target's center. For a vessel that's what you want;
        // for a moon it gives an SOI encounter with a low (possibly impact)
        // periapsis that needs a small correction after.
        // ------------------------------------------------------------------
        MacroStart StartInterceptCalc(bool fromQueue)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            bool havePlan = interceptPlannedNode != null && vessel.patchedConicSolver.maneuverNodes.Contains(interceptPlannedNode);
            if (havePlan && !fromQueue)
            {
                ManeuverNode planned = interceptPlannedNode;
                interceptPlannedNode = null;
                StartInterceptJob(vessel, planned);
                return MacroStart.Started;
            }

            // A plan made before the queue got here is stale - replan.
            if (havePlan)
                vessel.patchedConicSolver.RemoveManeuverNode(interceptPlannedNode);
            interceptPlannedNode = null;

            Orbit targetOrbit = GetTargetOrbitInSameSOI(vessel, "INTERCEPT");
            if (targetOrbit == null)
            {
                SetLEDState(0x04, LEDStates.INTERCEPT_IDLE, 0x00);
                return MacroStart.Failed;
            }

            SetLEDState(0x04, LEDStates.INTERCEPT_CALCULATING, 0x00);

            CelestialBody body = vessel.mainBody;
            double minSafeRadius = body.Radius + (body.atmosphere ? body.atmosphereDepth : 0.0) + INTERCEPT_SAFE_ALTITUDE_MARGIN;

            if (!FindBestIntercept(vessel.orbit, targetOrbit, minSafeRadius, out double departureUT, out Vector3d dv))
            {
                Debug.Log("[KSPMacropad] INTERCEPT: no intercept found in the search window");
                SetLEDState(0x04, LEDStates.INTERCEPT_INSUFFICIENT_DV, 0x00);
                return MacroStart.Failed;
            }

            Debug.Log("[KSPMacropad] INTERCEPT: burn of " + dv.magnitude + " m/s at UT " + departureUT);

            if (!HasDeltaVFor(vessel, dv.magnitude, "INTERCEPT"))
            {
                SetLEDState(0x04, LEDStates.INTERCEPT_INSUFFICIENT_DV, 0x00);
                return MacroStart.Failed;
            }

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(departureUT);
            SetNodeFromOrbitFrameDv(vessel, node, dv);

            if (fromQueue)
            {
                StartInterceptJob(vessel, node);
                return MacroStart.Started;
            }

            interceptPlannedNode = node;
            SetLEDState(0x04, LEDStates.INTERCEPT_SOLUTION, 0x00);
            return MacroStart.Done;
        }

        void StartInterceptJob(Vessel vessel, ManeuverNode node)
        {
            StartNodeJob(vessel, node, 0x04, LEDStates.INTERCEPT_IDLE, LEDStates.INTERCEPT_SOLUTION, LEDStates.INTERCEPT_SOLUTION,
                () =>
                {
                    SetLEDState(0x04, LEDStates.INTERCEPT_IDLE, 0x00);
                    OnMacroFinished(0x04, true);
                });
        }

        // Grid search over departure time (up to one synodic period, capped
        // at INTERCEPT_MAX_WINDOW_ORBITS vessel orbits) x time of flight,
        // Lambert-solving each pair and keeping the smallest departure burn.
        // Rejects any transfer orbit whose periapsis is below minSafeRadius
        // - conservative, since that periapsis might fall after the
        // intercept, but it never returns a trajectory through the ground.
        bool FindBestIntercept(Orbit vesselOrbit, Orbit targetOrbit, double minSafeRadius,
            out double bestDepartureUT, out Vector3d bestDv)
        {
            bestDepartureUT = 0;
            bestDv = Vector3d.zero;
            double bestCost = double.MaxValue;

            double mu = vesselOrbit.referenceBody.gravParameter;
            double start = Planetarium.GetUniversalTime() + NODE_MIN_LEAD_SECONDS;
            double window = Math.Min(OrbitMath.SynodicPeriod(vesselOrbit.period, targetOrbit.period),
                INTERCEPT_MAX_WINDOW_ORBITS * vesselOrbit.period);

            double aTransfer = (vesselOrbit.semiMajorAxis + targetOrbit.semiMajorAxis) / 2.0;
            double hohmannEstimate = Math.PI * Math.Sqrt(Math.Pow(aTransfer, 3) / mu);

            for (int i = 0; i < INTERCEPT_DEPARTURE_SAMPLES; i++)
            {
                double departureUT = start + window * i / (INTERCEPT_DEPARTURE_SAMPLES - 1);
                Vector3d r1 = vesselOrbit.getRelativePositionAtUT(departureUT);
                Vector3d v0 = vesselOrbit.getOrbitalVelocityAtUT(departureUT);
                Vector3d refNormal = Vector3d.Cross(r1, v0);

                for (int j = 0; j < INTERCEPT_TOF_SAMPLES; j++)
                {
                    double frac = INTERCEPT_TOF_MIN_FRACTION +
                        (INTERCEPT_TOF_MAX_FRACTION - INTERCEPT_TOF_MIN_FRACTION) * j / (INTERCEPT_TOF_SAMPLES - 1);
                    double tof = hohmannEstimate * frac;
                    Vector3d r2 = targetOrbit.getRelativePositionAtUT(departureUT + tof);

                    if (!OrbitMath.SolveLambert(r1, r2, tof, mu, refNormal, out Vector3d v1, out Vector3d _))
                        continue;

                    if (OrbitMath.PeriapsisRadius(r1, v1, mu) < minSafeRadius)
                        continue;

                    double cost = (v1 - v0).magnitude;
                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestDepartureUT = departureUT;
                        bestDv = v1 - v0;
                    }
                }
            }

            return bestCost < double.MaxValue;
        }

        // ------------------------------------------------------------------
        // ORBIT SYNC (0x05): match the target's orbital plane, then its
        // period. Plane change happens where the vessel next crosses the
        // target's plane (found numerically, so no ascending/descending node
        // bookkeeping); the period match is a prograde/retrograde burn at
        // the following periapsis that sets semi-major axis = target's.
        // A second press while either burn is pending cancels it.
        // ------------------------------------------------------------------
        MacroStart StartOrbitSync()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            Orbit targetOrbit = GetTargetOrbitInSameSOI(vessel, "ORBIT SYNC");
            if (targetOrbit == null)
            {
                SetLEDState(0x05, LEDStates.ORBSYNC_IDLE, 0x00);
                return MacroStart.Failed;
            }

            SetLEDState(0x05, LEDStates.ORBSYNC_CALCULATING, 0x00);

            Orbit orbit = vessel.orbit;
            double now = Planetarium.GetUniversalTime();
            Vector3d targetNormal = OrbitNormalAt(targetOrbit, now);

            if (OrbitMath.AngleDeg(OrbitNormalAt(orbit, now), targetNormal) < ORBITSYNC_MIN_REL_INCLINATION_DEG)
                return StartOrbitSyncPeriodMatch(targetOrbit);

            double searchStart = now + NODE_MIN_LEAD_SECONDS;
            bool found = OrbitMath.FindFirstSignChange(
                t => Vector3d.Dot(orbit.getRelativePositionAtUT(t), targetNormal),
                searchStart, searchStart + orbit.period, ORBITSYNC_CROSSING_SAMPLES, out double crossingUT);

            if (!found)
            {
                Debug.Log("[KSPMacropad] ORBIT SYNC: couldn't find where the orbit crosses the target's plane");
                SetLEDState(0x05, LEDStates.ORBSYNC_IDLE, 0x00);
                return MacroStart.Failed;
            }

            Vector3d dv = OrbitMath.PlaneMatchDv(orbit.getOrbitalVelocityAtUT(crossingUT), targetNormal);
            if (!HasDeltaVFor(vessel, dv.magnitude, "ORBIT SYNC"))
            {
                SetLEDState(0x05, LEDStates.ORBSYNC_IDLE, 0x00);
                return MacroStart.Failed;
            }

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(crossingUT);
            SetNodeFromOrbitFrameDv(vessel, node, dv);

            StartNodeJob(vessel, node, 0x05, LEDStates.ORBSYNC_IDLE, LEDStates.ORBSYNC_EXECUTING, LEDStates.ORBSYNC_EXECUTING,
                () =>
                {
                    MacroStart periodMatch = StartOrbitSyncPeriodMatch(targetOrbit);
                    if (periodMatch != MacroStart.Started)
                        OnMacroFinished(0x05, periodMatch == MacroStart.Done);
                });
            return MacroStart.Started;
        }

        MacroStart StartOrbitSyncPeriodMatch(Orbit targetOrbit)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            Orbit orbit = vessel.orbit;
            double mu = orbit.referenceBody.gravParameter;
            double now = Planetarium.GetUniversalTime();

            double burnUT = now + orbit.timeToPe;
            if (burnUT < now + NODE_MIN_LEAD_SECONDS)
                burnUT += orbit.period;

            Vector3d pos = orbit.getRelativePositionAtUT(burnUT);
            Vector3d vel = orbit.getOrbitalVelocityAtUT(burnUT);
            double r = pos.magnitude;
            double aTarget = targetOrbit.semiMajorAxis;

            if (2.0 / r - 1.0 / aTarget <= 0)
            {
                Debug.Log("[KSPMacropad] ORBIT SYNC: target's orbit is too small to reach from this periapsis");
                SetLEDState(0x05, LEDStates.ORBSYNC_IDLE, 0x00);
                return MacroStart.Failed;
            }

            Vector3d dv = vel.normalized * OrbitMath.VisVivaSpeed(mu, r, aTarget) - vel;
            if (dv.magnitude < AUTOPILOT_DV_EPSILON)
            {
                SetLEDState(0x05, LEDStates.ORBSYNC_COMPLETE, 0x00);
                return MacroStart.Done;
            }

            if (!HasDeltaVFor(vessel, dv.magnitude, "ORBIT SYNC"))
            {
                SetLEDState(0x05, LEDStates.ORBSYNC_IDLE, 0x00);
                return MacroStart.Failed;
            }

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(burnUT);
            SetNodeFromOrbitFrameDv(vessel, node, dv);

            StartNodeJob(vessel, node, 0x05, LEDStates.ORBSYNC_IDLE, LEDStates.ORBSYNC_EXECUTING, LEDStates.ORBSYNC_EXECUTING,
                () =>
                {
                    SetLEDState(0x05, LEDStates.ORBSYNC_COMPLETE, 0x00);
                    OnMacroFinished(0x05, true);
                });
            return MacroStart.Started;
        }

        // ------------------------------------------------------------------
        // RENDEZVOUS PREP (0x06): toggle. On: SAS + RCS on, SAS pointed at
        // the target, precision control on. While on, streams closing
        // velocity to the pad through the LED data byte (0-255, scaled to
        // RENDEZVOUS_MAX_CLOSING_MS; 0 when separating) and switches to
        // IN_RANGE inside RENDEZVOUS_IN_RANGE_METERS. Off restores the
        // previous precision-control setting.
        // ------------------------------------------------------------------
        void DoRendezvousPrep()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (rendezvousActive)
            {
                rendezvousActive = false;
                FlightInputHandler.fetch.precisionMode = rendezvousPrevPrecisionMode;
                SetLEDState(0x06, LEDStates.RENDEZVOUS_IDLE, 0x00);
                return;
            }

            EnsureSAS(vessel);
            if (!vessel.ActionGroups[KSPActionGroup.RCS])
                vessel.ActionGroups.ToggleGroup(KSPActionGroup.RCS);

            if (FlightGlobals.fetch.VesselTarget != null && vessel.Autopilot.CanSetMode(VesselAutopilot.AutopilotMode.Target))
                vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Target);

            rendezvousPrevPrecisionMode = FlightInputHandler.fetch.precisionMode;
            FlightInputHandler.fetch.precisionMode = true;

            rendezvousActive = true;
            SetLEDState(0x06, LEDStates.RENDEZVOUS_ACTIVE, 0x00);
        }

        void TickRendezvous()
        {
            if (!rendezvousActive)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target == null)
            {
                SetLEDState(0x06, LEDStates.RENDEZVOUS_ACTIVE, 0x00);
                return;
            }

            Vector3d relPos = ToVector3d(target.GetTransform().position) - vessel.GetWorldPos3D();
            Vector3d relVel = vessel.obt_velocity - target.GetObtVelocity();
            double distance = relPos.magnitude;
            double closing = distance > 0 ? Vector3d.Dot(relVel, relPos / distance) : 0.0;

            double scaled = Math.Max(0.0, Math.Min(1.0, closing / RENDEZVOUS_MAX_CLOSING_MS));
            byte data = (byte)Math.Round(scaled * 255.0);
            byte state = distance < RENDEZVOUS_IN_RANGE_METERS ? LEDStates.RENDEZVOUS_IN_RANGE : LEDStates.RENDEZVOUS_ACTIVE;

            SetLEDState(0x06, state, data);
        }

        // ------------------------------------------------------------------
        // AUTO GRAVITY TURN (0x01): pitches from AGT_START_PITCH_DEG
        // at AGT_START_ALTITUDE down to AGT_END_PITCH_DEG at AGT_END_ALTITUDE
        // (linear in altitude), heading due east. While dynamic pressure is
        // above AGT_MAX_Q_KPA the commanded direction is held within
        // AGT_MAX_AOA_DEG of surface prograde to keep aero loads down. Cuts
        // throttle and ends once apoapsis reaches AGT_TARGET_APOAPSIS,
        // leaving the circularization to CIRCULARIZE (queue it to have that
        // happen automatically). Doesn't touch the throttle otherwise.
        // ------------------------------------------------------------------
        MacroStart StartAutoGravityTurn()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return MacroStart.Failed;

            Vessel.Situations s = vessel.situation;
            if (s != Vessel.Situations.PRELAUNCH && s != Vessel.Situations.FLYING && s != Vessel.Situations.SUB_ORBITAL)
            {
                Debug.Log("[KSPMacropad] AUTO GRAVITY TURN: only runs during ascent");
                return MacroStart.Failed;
            }

            EnsureSAS(vessel);
            vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
            agtActive = true;
            SetLEDState(0x01, LEDStates.AGT_ACTIVE, 0x00);
            return MacroStart.Started;
        }

        void TickGravityTurn()
        {
            if (!agtActive)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (vessel.situation == Vessel.Situations.LANDED || vessel.situation == Vessel.Situations.SPLASHED)
            {
                StopGravityTurn(vessel, "vessel is back on the ground", false);
                return;
            }

            if (vessel.situation == Vessel.Situations.ORBITING || vessel.situation == Vessel.Situations.ESCAPING)
            {
                StopGravityTurn(vessel, "reached orbit", true);
                return;
            }

            if (vessel.orbit.ApA >= AGT_TARGET_APOAPSIS)
            {
                FlightInputHandler.state.mainThrottle = 0f;
                StopGravityTurn(vessel, "target apoapsis reached", true);
                return;
            }

            double pitchDeg = OrbitMath.GravityTurnPitch(vessel.altitude, AGT_START_ALTITUDE, AGT_END_ALTITUDE,
                AGT_START_PITCH_DEG, AGT_END_PITCH_DEG);

            LocalFrame(vessel, out Vector3d up, out Vector3d north, out Vector3d east);
            OrbitMath.HeadingPitchDirection(up, north, east, pitchDeg, 90.0, out Vector3d direction, out Vector3d rollReference);

            Vector3d srfVelocity = ToVector3d(vessel.GetSrfVelocity());
            if (vessel.rootPart != null && vessel.rootPart.dynamicPressurekPa > AGT_MAX_Q_KPA &&
                srfVelocity.magnitude > AGT_MIN_SRF_SPEED_FOR_AOA)
            {
                direction = OrbitMath.LimitAngle(srfVelocity, direction, AGT_MAX_AOA_DEG);
            }

            vessel.Autopilot.SAS.LockRotation(NoseRotation(direction, rollReference));
        }

        // World-space up/north/east at the vessel. Cross(up, north) is east
        // in Unity's axes (x right when y is up and z is forward).
        static void LocalFrame(Vessel vessel, out Vector3d up, out Vector3d north, out Vector3d east)
        {
            up = (vessel.CoMD - vessel.mainBody.position).normalized;
            Vector3d axis = ToVector3d(vessel.mainBody.transform.up);
            north = (axis - up * Vector3d.Dot(axis, up)).normalized;
            east = Vector3d.Cross(up, north);
        }

        // SAS rotation that points the vessel's nose along `direction`. The
        // nose is ReferenceTransform.up, not forward, so LookRotation (which
        // aims forward) is followed by a 90 degree pitch that maps local up
        // onto the aimed direction. rollReference fixes the roll.
        static Quaternion NoseRotation(Vector3d direction, Vector3d rollReference)
        {
            return Quaternion.LookRotation((Vector3)direction, (Vector3)rollReference) * Quaternion.Euler(90f, 0f, 0f);
        }

        void StopGravityTurn(Vessel vessel, string reason, bool success)
        {
            Debug.Log("[KSPMacropad] AUTO GRAVITY TURN: stopped - " + reason);
            agtActive = false;
            if (vessel != null)
                vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
            SetLEDState(0x01, LEDStates.AGT_IDLE, 0x00);
            OnMacroFinished(0x01, success);
        }

        // ------------------------------------------------------------------
        // Encoders. Mode 1 is live control; modes 2 and 3 dial values that
        // are applied when their key is pressed again (the firmware drops
        // back to mode 1 at the same moment). Pushing the encoder shafts
        // isn't wired on this board (no free pins), so there's no click
        // handling here.
        // ------------------------------------------------------------------
        void HandleEncoder(int mode, int side, int steps)
        {
            if (mode != encoderMode)
            {
                Debug.Log("[KSPMacropad] Encoder mode resynced to " + mode + " from the pad (was " + encoderMode + ")");
                encoderMode = mode;
            }

            switch (mode)
            {
                case 1:
                    if (side == 1) AdjustThrottle(steps);
                    else AdjustWarp(steps);
                    break;

                case 2:
                    if (side == 1)
                    {
                        precisionThrottleTarget = Math.Max(0, Math.Min(100, precisionThrottleTarget + steps));
                        precisionThrottleDialed = true;
                    }
                    else
                    {
                        precisionHeadingTarget = OrbitMath.WrapDegrees(precisionHeadingTarget + steps);
                        precisionHeadingDialed = true;
                    }
                    break;

                case 3:
                    if (side == 1)
                    {
                        AdjustZoom(steps);
                    }
                    else
                    {
                        rcsLimiterTarget = Math.Max(0, Math.Min(100, rcsLimiterTarget + steps));
                        rcsLimiterDialed = true;
                    }
                    break;
            }
        }

        // Same toggles as the firmware: pressing a mode's key while in that
        // mode commits it and returns to mode 1; pressing it from any other
        // mode switches straight to it (an uncommitted mode is dropped).
        void OnPrecisionInputKey()
        {
            if (encoderMode == 2)
            {
                CommitPrecisionInput();
                encoderMode = 1;
                return;
            }

            encoderMode = 2;
            precisionThrottleDialed = false;
            precisionHeadingDialed = false;
        }

        void OnAuxModeKey()
        {
            if (encoderMode == 3)
            {
                CommitAuxMode();
                encoderMode = 1;
                return;
            }

            encoderMode = 3;
            rcsLimiterDialed = false;
        }

        void AdjustThrottle(int steps)
        {
            float throttle = FlightInputHandler.state.mainThrottle + steps * THROTTLE_STEP_PER_DETENT;
            FlightInputHandler.state.mainThrottle = Math.Max(0f, Math.Min(1f, throttle));
        }

        // On-rails warp is capped by KSP's own altitude limit for the
        // current body; physics warp by its rate table.
        void AdjustWarp(int steps)
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            TimeWarp warp = TimeWarp.fetch;
            if (vessel == null || warp == null)
                return;

            int maxIndex;
            if (TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
                maxIndex = Math.Min(warp.warpRates.Length - 1, warp.GetMaxRateForAltitude(vessel.altitude, vessel.mainBody));
            else
                maxIndex = warp.physicsWarpRates.Length - 1;

            int target = Math.Max(0, Math.Min(maxIndex, TimeWarp.CurrentRateIndex + steps));
            if (target != TimeWarp.CurrentRateIndex)
                TimeWarp.SetRate(target, false);
        }

        // Clockwise (positive steps) zooms in.
        void AdjustZoom(int steps)
        {
            FlightCamera camera = FlightCamera.fetch;
            if (camera == null)
                return;

            double distance = camera.Distance * Math.Pow(ZOOM_FACTOR_PER_DETENT, -steps);
            distance = Math.Max(camera.minDistance, Math.Min(camera.maxDistance, distance));
            camera.SetDistance((float)distance);
        }

        void CommitPrecisionInput()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (precisionThrottleDialed)
            {
                FlightInputHandler.state.mainThrottle = precisionThrottleTarget / 100f;
                Debug.Log("[KSPMacropad] PRECISION: throttle set to " + precisionThrottleTarget + "%");
            }

            if (!precisionHeadingDialed)
                return;

            if (IsFlightComputerBusy())
            {
                Debug.Log("[KSPMacropad] PRECISION: " + MacroName(activeMacroKey) + " is steering, heading not applied");
                return;
            }

            Vessel.Situations s = vessel.situation;
            if (s == Vessel.Situations.PRELAUNCH || s == Vessel.Situations.LANDED || s == Vessel.Situations.SPLASHED)
            {
                Debug.Log("[KSPMacropad] PRECISION: heading hold only works in flight");
                return;
            }

            if (!vessel.Autopilot.CanSetMode(VesselAutopilot.AutopilotMode.StabilityAssist))
            {
                Debug.Log("[KSPMacropad] PRECISION: vessel has no SAS, heading not applied");
                return;
            }

            // Hold the current pitch; only the heading changes.
            LocalFrame(vessel, out Vector3d up, out Vector3d north, out Vector3d east);
            Vector3d nose = ToVector3d(vessel.ReferenceTransform.up);
            headingHoldPitchDeg = 90.0 - OrbitMath.AngleDeg(up, nose);
            headingHoldHeadingDeg = precisionHeadingTarget;
            headingHoldActive = true;

            EnsureSAS(vessel);
            vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.StabilityAssist);
            Debug.Log("[KSPMacropad] PRECISION: holding heading " + precisionHeadingTarget + " at pitch " +
                headingHoldPitchDeg.ToString("F1"));
        }

        // Re-locks SAS onto the held heading every frame (the local frame
        // turns as the vessel moves around the body). Releases on manual
        // pitch/yaw/roll input, when a flying macro takes over, or on landing.
        void TickHeadingHold()
        {
            if (!headingHoldActive)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            FlightCtrlState input = FlightInputHandler.state;
            string releaseReason = null;
            if (Math.Abs(input.pitch) > MANUAL_INPUT_THRESHOLD || Math.Abs(input.yaw) > MANUAL_INPUT_THRESHOLD ||
                Math.Abs(input.roll) > MANUAL_INPUT_THRESHOLD)
                releaseReason = "manual input";
            else if (IsFlightComputerBusy())
                releaseReason = MacroName(activeMacroKey) + " took over";
            else if (vessel.situation == Vessel.Situations.LANDED || vessel.situation == Vessel.Situations.SPLASHED)
                releaseReason = "landed";

            if (releaseReason != null)
            {
                headingHoldActive = false;
                Debug.Log("[KSPMacropad] PRECISION: heading hold released - " + releaseReason);
                return;
            }

            LocalFrame(vessel, out Vector3d up, out Vector3d north, out Vector3d east);
            OrbitMath.HeadingPitchDirection(up, north, east, headingHoldPitchDeg, headingHoldHeadingDeg,
                out Vector3d direction, out Vector3d rollReference);
            vessel.Autopilot.SAS.LockRotation(NoseRotation(direction, rollReference));
        }

        void CommitAuxMode()
        {
            if (!rcsLimiterDialed)
                return;

            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            List<ModuleRCS> thrusters = vessel.FindPartModulesImplementing<ModuleRCS>();
            foreach (ModuleRCS rcs in thrusters)
                rcs.thrustPercentage = rcsLimiterTarget;

            Debug.Log("[KSPMacropad] AUX: RCS thrust limiter set to " + rcsLimiterTarget + "% on " + thrusters.Count + " thruster(s)");
        }

        // VesselDeltaV.TotalDeltaVActual: "The Total Simulated DeltaV
        // produced by the Vessel/Ship in flight" (confirmed in the KSP 1.x
        // API docs). Returns 0 until the stock simulation has run once.
        double GetVesselDeltaV(Vessel vessel)
        {
            if (vessel.VesselDeltaV == null)
                return 0.0;

            return vessel.VesselDeltaV.TotalDeltaVActual;
        }

    }

    // Pure orbital-mechanics math with no KSP state access, so it can be
    // compiled and unit-tested outside the game. Every vector passed in
    // must be in one consistent frame (the Orbit frame, in practice).
    internal static class OrbitMath
    {
        public static double CircularSpeed(double mu, double r)
        {
            return Math.Sqrt(mu / r);
        }

        public static double VisVivaSpeed(double mu, double r, double a)
        {
            return Math.Sqrt(mu * (2.0 / r - 1.0 / a));
        }

        public static double SynodicPeriod(double periodA, double periodB)
        {
            if (periodA <= 0 || periodB <= 0 || Math.Abs(periodA - periodB) < 1e-6)
                return Math.Max(periodA, periodB); // degenerate - fall back rather than dividing by ~0
            return Math.Abs(1.0 / (1.0 / periodA - 1.0 / periodB));
        }

        public static double AngleDeg(Vector3d a, Vector3d b)
        {
            double d = Vector3d.Dot(a.normalized, b.normalized);
            return Math.Acos(Math.Max(-1.0, Math.Min(1.0, d))) * 180.0 / Math.PI;
        }

        public static Vector3d SwapYZ(Vector3d v)
        {
            return new Vector3d(v.x, v.z, v.y);
        }

        // Unit vector along the velocity with its radial component removed.
        public static Vector3d Horizontal(Vector3d pos, Vector3d vel)
        {
            Vector3d rHat = pos.normalized;
            return (vel - rHat * Vector3d.Dot(vel, rHat)).normalized;
        }

        // dv that rotates vel into the plane with the given normal, keeping
        // its magnitude. Sign of planeNormal doesn't matter.
        public static Vector3d PlaneMatchDv(Vector3d vel, Vector3d planeNormal)
        {
            Vector3d n = planeNormal.normalized;
            Vector3d inPlane = vel - n * Vector3d.Dot(vel, n);
            return inPlane.normalized * vel.magnitude - vel;
        }

        // Splits dv into radial-out (perpendicular to velocity, in the
        // orbital plane), normal (along pos x vel) and prograde components.
        public static void DecomposeBurn(Vector3d pos, Vector3d vel, Vector3d dv,
            out double radial, out double normal, out double prograde)
        {
            Vector3d pro = vel.normalized;
            Vector3d rad = (pos - pro * Vector3d.Dot(pos, pro)).normalized;
            Vector3d nrm = Vector3d.Cross(pos, vel).normalized;
            radial = Vector3d.Dot(dv, rad);
            normal = Vector3d.Dot(dv, nrm);
            prograde = Vector3d.Dot(dv, pro);
        }

        // Periapsis radius of the conic through (pos, vel): p / (1 + e).
        // Valid for elliptic, parabolic and hyperbolic orbits alike.
        public static double PeriapsisRadius(Vector3d pos, Vector3d vel, double mu)
        {
            double r = pos.magnitude;
            double energy = Vector3d.Dot(vel, vel) / 2.0 - mu / r;
            double h = Vector3d.Cross(pos, vel).magnitude;
            double e = Math.Sqrt(Math.Max(0.0, 1.0 + 2.0 * energy * h * h / (mu * mu)));
            return h * h / (mu * (1.0 + e));
        }

        // Returns `to`, or if it's more than maxDeg from `from`, the unit
        // vector maxDeg away from `from` toward `to`.
        public static Vector3d LimitAngle(Vector3d from, Vector3d to, double maxDeg)
        {
            Vector3d f = from.normalized;
            Vector3d t = to.normalized;
            if (AngleDeg(f, t) <= maxDeg)
                return t;

            Vector3d perp = t - f * Vector3d.Dot(t, f);
            if (perp.magnitude < 1e-9)
                return f; // exactly opposite - no defined direction to rotate toward

            double m = maxDeg * Math.PI / 180.0;
            return f * Math.Cos(m) + perp.normalized * Math.Sin(m);
        }

        // Wraps any integer degree value into 0-359 (C#'s % keeps the sign).
        public static int WrapDegrees(int degrees)
        {
            return ((degrees % 360) + 360) % 360;
        }

        // Unit direction at the given pitch above the horizon and compass
        // heading (0 = north, 90 = east), plus a horizontal vector
        // perpendicular to it to fix the roll. up/north/east must be an
        // orthonormal local frame.
        public static void HeadingPitchDirection(Vector3d up, Vector3d north, Vector3d east, double pitchDeg,
            double headingDeg, out Vector3d direction, out Vector3d rollReference)
        {
            double p = pitchDeg * Math.PI / 180.0;
            double h = headingDeg * Math.PI / 180.0;
            Vector3d horizontal = north * Math.Cos(h) + east * Math.Sin(h);
            direction = up * Math.Sin(p) + horizontal * Math.Cos(p);
            rollReference = north * Math.Sin(h) - east * Math.Cos(h);
        }

        public static double GravityTurnPitch(double altitude, double startAltitude, double endAltitude,
            double startPitch, double endPitch)
        {
            if (altitude <= startAltitude)
                return startPitch;
            if (altitude >= endAltitude)
                return endPitch;
            return startPitch + (endPitch - startPitch) * (altitude - startAltitude) / (endAltitude - startAltitude);
        }

        // Scans [t0, t1] in `samples` steps for the first sign change of f
        // and bisects it down. Returns false if f never changes sign.
        public static bool FindFirstSignChange(Func<double, double> f, double t0, double t1, int samples, out double root)
        {
            root = 0;
            double prevT = t0;
            double prevF = f(t0);

            for (int i = 1; i <= samples; i++)
            {
                double t = t0 + (t1 - t0) * i / samples;
                double ft = f(t);

                if ((prevF < 0) != (ft < 0))
                {
                    double lo = prevT, hi = t, fLo = prevF;
                    for (int k = 0; k < 60; k++)
                    {
                        double mid = (lo + hi) / 2.0;
                        double fMid = f(mid);
                        if ((fMid < 0) == (fLo < 0))
                        {
                            lo = mid;
                            fLo = fMid;
                        }
                        else
                        {
                            hi = mid;
                        }
                    }
                    root = (lo + hi) / 2.0;
                    return true;
                }

                prevT = t;
                prevF = ft;
            }

            return false;
        }

        // Universal-variable Lambert solver (single revolution, prograde
        // with respect to refNormal). Solves for the transfer-orbit
        // velocities connecting r1vec to r2vec in time tof. Finds the root
        // of the Stumpff-function time equation by scanning the valid
        // z-range for a sign change and bisecting, rather than Newton with a
        // hand-derived derivative. Returns false for a near-180-degree
        // transfer angle (singular for this method) or if no root is found.
        public static bool SolveLambert(Vector3d r1vec, Vector3d r2vec, double tof, double mu, Vector3d refNormal,
            out Vector3d v1, out Vector3d v2)
        {
            v1 = Vector3d.zero;
            v2 = Vector3d.zero;

            double r1 = r1vec.magnitude;
            double r2 = r2vec.magnitude;

            double cosDnu = Vector3d.Dot(r1vec, r2vec) / (r1 * r2);
            cosDnu = Math.Max(-1.0, Math.Min(1.0, cosDnu));
            double dnu = Math.Acos(cosDnu);
            if (Vector3d.Dot(Vector3d.Cross(r1vec, r2vec), refNormal) < 0)
                dnu = 2.0 * Math.PI - dnu;

            double A = Math.Sin(dnu) * Math.Sqrt(r1 * r2 / (1.0 - Math.Cos(dnu)));
            if (Math.Abs(A) < 1e-6)
                return false;

            double zScanLo = -4.0 * Math.PI * Math.PI + 1e-6;
            double zScanHi = 4.0 * Math.PI * Math.PI - 1e-6;
            const int scanSteps = 400;

            double prevZ = 0, prevF = 0;
            bool havePrev = false;
            double bracketLoZ = 0, bracketLoF = 0, bracketHiZ = 0;
            bool haveBracket = false;

            for (int i = 0; i <= scanSteps; i++)
            {
                double z = zScanLo + (zScanHi - zScanLo) * i / scanSteps;
                if (!LambertF(z, r1, r2, A, mu, tof, out double Fz))
                {
                    havePrev = false;
                    continue;
                }

                if (havePrev && (prevF < 0) != (Fz < 0))
                {
                    bracketLoZ = prevZ;
                    bracketLoF = prevF;
                    bracketHiZ = z;
                    haveBracket = true;
                    break;
                }

                prevZ = z;
                prevF = Fz;
                havePrev = true;
            }

            if (!haveBracket)
                return false;

            double zLow = bracketLoZ, fLow = bracketLoF;
            double zHigh = bracketHiZ;

            for (int i = 0; i < 100; i++)
            {
                double zMid = (zLow + zHigh) / 2.0;
                if (!LambertF(zMid, r1, r2, A, mu, tof, out double fMid))
                {
                    zHigh = zMid;
                    continue;
                }
                if (Math.Abs(fMid) < 1e-6)
                {
                    zLow = zHigh = zMid;
                    break;
                }
                if ((fMid < 0) == (fLow < 0))
                {
                    zLow = zMid;
                    fLow = fMid;
                }
                else
                {
                    zHigh = zMid;
                }
            }

            double zFinal = (zLow + zHigh) / 2.0;
            double yFinal = LambertY(zFinal, r1, r2, A);
            if (yFinal < 0)
                return false;

            double f = 1.0 - yFinal / r1;
            double g = A * Math.Sqrt(yFinal / mu);
            double gDot = 1.0 - yFinal / r2;

            v1 = (r2vec - f * r1vec) / g;
            v2 = (gDot * r2vec - r1vec) / g;
            return true;
        }

        static double LambertY(double z, double r1, double r2, double A)
        {
            return r1 + r2 + A * (z * StumpffS(z) - 1.0) / Math.Sqrt(StumpffC(z));
        }

        static bool LambertF(double z, double r1, double r2, double A, double mu, double tof, out double F)
        {
            F = 0;
            double y = LambertY(z, r1, r2, A);
            if (y < 0)
                return false;

            F = Math.Pow(y / StumpffC(z), 1.5) * StumpffS(z) + A * Math.Sqrt(y) - Math.Sqrt(mu) * tof;
            return true;
        }

        static double StumpffC(double z)
        {
            if (z > 1e-6)
                return (1.0 - Math.Cos(Math.Sqrt(z))) / z;
            if (z < -1e-6)
                return (Math.Cosh(Math.Sqrt(-z)) - 1.0) / (-z);
            return 0.5 - z / 24.0 + z * z / 720.0;
        }

        static double StumpffS(double z)
        {
            if (z > 1e-6)
            {
                double sz = Math.Sqrt(z);
                return (sz - Math.Sin(sz)) / (sz * sz * sz);
            }
            if (z < -1e-6)
            {
                double sz = Math.Sqrt(-z);
                return (Math.Sinh(sz) - sz) / (sz * sz * sz);
            }
            return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0;
        }
    }
}