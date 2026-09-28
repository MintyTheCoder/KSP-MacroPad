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

        // AUTOPILOT state machine (see DoAutopilot/TickAutopilot below).
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
                                DoAutopilot();
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
            TickAutopilot();
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

            // Iterates the real tracked-key list (not a raw 0x00-0x0D range),
            // since 0x0B has no LED states and 0x0F (AUTOPILOT) does.
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
        // Destination timing/magnitude are now solved for real via
        // FindBestTransferWindow()'s Lambert search below, rather than
        // assumed - see that method for what it actually searches.
        //
        // Known remaining simplifying assumptions (first-pass, not tuned/
        // verified in-game):
        //   - ejection and capture burns are executed prograde/retrograde
        //     only. The true hyperbolic-excess-velocity vector the Lambert
        //     solve produces generally isn't purely prograde at the burn
        //     point - it can have a real radial/normal component. Using
        //     only the magnitude (not the full 3D direction) of vInf is a
        //     real source of residual pointing error this build doesn't
        //     correct for.
        //   - no SOI-transit-time offset: a real patched-conic plan burns
        //     somewhat before the nominal departure UT so the vessel is
        //     actually at that velocity by the time it crosses the SOI
        //     boundary. This build burns AT the found departure UT instead.
        //   - the search grid (vessel's own periapsis passages x a handful
        //     of time-of-flight samples per passage) is coarse, not a true
        //     continuous optimum - it's a reasonable window, not the
        //     cheapest possible one.
        //   - mid-course correction only nulls relative inclination between
        //     the vessel's transfer orbit and the destination's orbital
        //     plane, not a full aim-point correction for the pointing error
        //     from the two points above.
        //   - capture parking altitude is a flat +100km over the destination
        //     body's radius, not anything the player chose.
        //   - burn execution is full-throttle with no tapering near the end
        //     (node.GetBurnVector()'s magnitude is what's checked against
        //     AUTOPILOT_DV_EPSILON), so a high-TWR craft can overshoot.
        //   - no Principia (n-body) detection - doc already flags AUTOPILOT
        //     as broken under Principia; not checked for here.
        //   - Coordinate frame for FindBestTransferWindow's vector math:
        //     checked against the KSP API docs. getRelativePositionAtUT is
        //     documented as "all Vector3d's returned by Orbit class
        //     functions have their y and z axes flipped" - i.e. every Orbit
        //     method (including getOrbitalVelocityAtUT) is documented to
        //     share that same flipped convention, so subtracting two
        //     velocity vectors both read from Orbit methods (as this code
        //     does) should stay internally consistent. That note wasn't
        //     found written specifically against getOrbitalVelocityAtUT
        //     itself, only stated generally for the class, so treat this as
        //     "likely fine, not independently confirmed" rather than fully
        //     settled - recheck once this can actually run against KSP.
        //   - mid-course correction's burn direction is a known-unsolved
        //     sign question, not just unverified - see the comment on
        //     BuildMidCourseCorrectionNode.
        //   - GetVesselDeltaV's exact stock field name (TotalDeltaVActual)
        //     could NOT be confirmed - see that method's comment.
        void DoAutopilot()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return;

            if (autopilotPhase != AutopilotPhase.Idle)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: transfer already in progress, ignoring press");
                return;
            }

            ITargetable target = FlightGlobals.fetch.VesselTarget;
            if (target == null)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: no target selected, nothing to plan against");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return;
            }

            CelestialBody destinationBody = target as CelestialBody;
            if (destinationBody == null && target is Vessel targetVessel)
                destinationBody = targetVessel.mainBody;

            if (destinationBody == null)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: target has no resolvable body");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return;
            }

            CelestialBody originBody = vessel.mainBody;

            if (destinationBody == originBody)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: target is in the same SOI, not an interplanetary case");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return;
            }

            // MVP only handles "vessel orbiting a planet, transferring to
            // another planet" - nested-moon-to-elsewhere cases (or a vessel
            // already heliocentric) aren't covered yet.
            if (originBody.referenceBody == null || originBody.referenceBody != Planetarium.fetch.Sun)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: origin body's parent isn't the Sun - unsupported case for this MVP");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IDLE, 0x00);
                return;
            }

            bool foundWindow = FindBestTransferWindow(vessel, originBody, destinationBody,
                out double departureUT, out double arrivalUT, out double ejectionDv, out double captureDv);

            if (!foundWindow)
            {
                Debug.Log("[KSPMacropad] AUTOPILOT: no valid transfer window found in the search window");
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IMPOSSIBLE, 0x00);
                return;
            }

            double requiredDv = ejectionDv + captureDv;
            double availableDv = GetVesselDeltaV(vessel);

            Debug.Log("[KSPMacropad] AUTOPILOT: departureUT=" + departureUT + " arrivalUT=" + arrivalUT +
                " required dv=" + requiredDv + " (ejection=" + ejectionDv + " capture=" + captureDv +
                ") available dv=" + availableDv);

            if (availableDv < requiredDv)
            {
                SetLEDState(0x0F, LEDStates.AUTOPILOT_IMPOSSIBLE, 0x00);
                return;
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
            TimeWarp.fetch.WarpTo(departureUT);
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
                    "(expected - this build has no phase-angle/launch-window targeting)");
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
                        TimeWarp.fetch.WarpTo(autopilotMidCourseUT);
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
                        double periapsisUT = currentUT + vessel.orbit.timeToPe; // confirmed real property - see FindBestTransferWindow's header comment
                        double rCapture = autopilotDestination.Radius + 100000.0;
                        double vCircCapture = Math.Sqrt(autopilotDestination.gravParameter / rCapture);
                        double vAtPeriapsis = vessel.orbit.getOrbitalVelocityAtUT(periapsisUT).magnitude; // confirmed real method (getOrbitalSpeedAtUT does NOT exist - caught by checking against KSP API docs)
                        double captureBurnDv = vCircCapture - vAtPeriapsis; // negative = retrograde burn

                        ManeuverNode captureNode = vessel.patchedConicSolver.AddManeuverNode(periapsisUT);
                        captureNode.DeltaV = new Vector3d(0, 0, captureBurnDv); // confirmed real field/convention
                        vessel.patchedConicSolver.UpdateFlightPlan(); // confirmed real method

                        autopilotActiveNode = captureNode;
                        autopilotPhase = AutopilotPhase.WaitCaptureBurn;
                        TimeWarp.fetch.WarpTo(periapsisUT);
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
                    }
                    break;
            }
        }

        // Shared burn executor for every AUTOPILOT phase: points the vessel
        // at the node's burn vector using stock SAS maneuver-hold (same
        // Autopilot.Enabled/SetMode pattern DoDockingPrep already uses for
        // Target mode), holds full throttle, and reports done once the
        // node's remaining burn vector drops under AUTOPILOT_DV_EPSILON.
        bool ExecuteNodeBurn(Vessel vessel, ManeuverNode node)
        {
            double remaining = node.GetBurnVector(vessel.orbit).magnitude;

            if (remaining < AUTOPILOT_DV_EPSILON)
            {
                FlightInputHandler.state.mainThrottle = 0f;
                vessel.Autopilot.Enabled = false;
                vessel.patchedConicSolver.RemoveManeuverNode(node);
                return true;
            }

            vessel.Autopilot.Enabled = true;
            vessel.Autopilot.SetMode(VesselAutopilot.AutopilotMode.Maneuver);
            FlightInputHandler.state.mainThrottle = 1f; // full throttle, no tapering near completion - see caveats above
            return false;
        }

        // Mid-course correction: nulls relative inclination between the
        // vessel's current (heliocentric) orbital plane and the destination
        // body's orbital plane. This is a real, computable correction, but
        // it is NOT a real aim-point/targeting correction - it doesn't know
        // whether the ejection burn above is actually going to intercept the
        // destination (see the big caveat on DoAutopilot). Returns null if
        // the mismatch is negligible, so the caller can skip straight to
        // waiting for the destination SOI.
        //
        // KNOWN GAP: the sign of normalDv below is a guess. Confirmed
        // against the KSP API docs: ManeuverNode.DeltaV's Y-component is
        // "delta-V in the normal-MINUS direction" (not plain normal), and
        // separately, this code never determines whether the vessel is
        // approaching its ascending or descending node relative to the
        // destination's orbital plane - that's what actually decides which
        // sign kills the mismatch instead of doubling it. So this burn may
        // apply in the wrong direction as-is; needs a real ascending/
        // descending-node check before trusting it.
        ManeuverNode BuildMidCourseCorrectionNode(Vessel vessel, CelestialBody destination, double ut)
        {
            double relIncDeg = Math.Abs(vessel.orbit.inclination - destination.orbit.inclination);
            if (relIncDeg < 0.05)
                return null;

            double relIncRad = relIncDeg * Math.PI / 180.0;
            double vNow = vessel.orbit.getOrbitalVelocityAtUT(ut).magnitude; // confirmed real method (getOrbitalSpeedAtUT does NOT exist)
            double normalDv = 2.0 * vNow * Math.Sin(relIncRad / 2.0);

            ManeuverNode node = vessel.patchedConicSolver.AddManeuverNode(ut);
            node.DeltaV = new Vector3d(0, normalDv, 0); // normal-only, no prograde component - see sign caveat above
            vessel.patchedConicSolver.UpdateFlightPlan(); // confirmed real method, no documented params
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
        }

        // ------------------------------------------------------------------
        // Transfer-window search (Lambert solver). This is the piece that
        // actually solves for WHEN to leave, not just how big a burn -
        // sizing a Hohmann-radius burn without this can put a vessel on an
        // orbit of the right size that still never meets the destination.
        //
        // The universal-variable Lambert solver below was prototyped and
        // checked in Python against a published textbook case (Curtis,
        // "Orbital Mechanics for Engineering Students", Example 5.2 - a
        // known r1/r2/time-of-flight triple with a known answer) before
        // being translated here, matching to 5 significant figures. The
        // C# translation itself has NOT been compiled or run (no KSP
        // install on this dev machine, same caveat as the rest of this
        // file) - re-check the arithmetic once it can actually build.
        // See DoAutopilot's comment for the specific coordinate-frame risk
        // this still carries (getRelativePositionAtUT vs
        // getOrbitalVelocityAtUT).
        // ------------------------------------------------------------------

        private const int TRANSFER_SEARCH_MAX_DEPARTURES = 60;
        private const int TRANSFER_SEARCH_TOF_SAMPLES = 8;
        private const double TRANSFER_SEARCH_TOF_MIN_FRACTION = 0.5;
        private const double TRANSFER_SEARCH_TOF_MAX_FRACTION = 1.5;

        // Searches the vessel's own upcoming periapsis passages (the burn
        // can only happen there, per the rest of this file) crossed with a
        // spread of times-of-flight around the analytic Hohmann estimate,
        // Lambert-solving each pair and keeping the one with the lowest
        // ejection+capture dv. Returns false if nothing in the grid
        // produced a valid Lambert solution.
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

            double r1Approx = originBody.orbit.semiMajorAxis;
            double r2Approx = destinationBody.orbit.semiMajorAxis;
            double aTransferApprox = (r1Approx + r2Approx) / 2.0;
            double hohmannEstimate = Math.PI * Math.Sqrt(Math.Pow(aTransferApprox, 3) / muSun);

            double synodicPeriod = EstimateSynodicPeriod(originBody.orbit.period, destinationBody.orbit.period);

            double currentUT = Planetarium.GetUniversalTime();
            double vesselPeriod = vessel.orbit.period;
            bool vesselPeriodValid = vesselPeriod > 0 && !double.IsNaN(vesselPeriod);

            int maxCandidates = vesselPeriodValid
                ? Math.Min(TRANSFER_SEARCH_MAX_DEPARTURES, (int)Math.Ceiling(synodicPeriod / vesselPeriod) + 1)
                : 1; // not on a stable elliptical orbit - only the immediate next periapsis is usable

            // Orbit has no NextPeriapsisTime(UT) method (checked against the
            // KSP API docs - it doesn't exist); timeToPe is the real,
            // confirmed property for this.
            double departureUT = currentUT + vessel.orbit.timeToPe;

            for (int d = 0; d < maxCandidates && departureUT < currentUT + synodicPeriod; d++)
            {
                Vector3d r1vec = originBody.orbit.getRelativePositionAtUT(departureUT); // confirmed real method
                Vector3d vOriginAtDep = originBody.orbit.getOrbitalVelocityAtUT(departureUT); // confirmed real method - see frame-consistency note on DoAutopilot

                for (int t = 0; t < TRANSFER_SEARCH_TOF_SAMPLES; t++)
                {
                    double frac = TRANSFER_SEARCH_TOF_MIN_FRACTION +
                        (TRANSFER_SEARCH_TOF_MAX_FRACTION - TRANSFER_SEARCH_TOF_MIN_FRACTION) * t / (TRANSFER_SEARCH_TOF_SAMPLES - 1);
                    double tof = hohmannEstimate * frac;
                    double arrivalUT = departureUT + tof;

                    Vector3d r2vec = destinationBody.orbit.getRelativePositionAtUT(arrivalUT);
                    Vector3d vDestAtArr = destinationBody.orbit.getOrbitalVelocityAtUT(arrivalUT);

                    if (!SolveLambert(r1vec, r2vec, tof, muSun, out Vector3d vTransferAtDep, out Vector3d vTransferAtArr))
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

        double EstimateSynodicPeriod(double periodA, double periodB)
        {
            if (periodA <= 0 || periodB <= 0 || Math.Abs(periodA - periodB) < 1e-6)
                return Math.Max(periodA, periodB); // degenerate - fall back rather than dividing by ~0
            return Math.Abs(1.0 / (1.0 / periodA - 1.0 / periodB));
        }

        // dv from a circular parking orbit around originBody up to
        // hyperbolic excess speed vInf (magnitude only - see the
        // pointing-error caveat on DoAutopilot).
        double VInfToEjectionDv(Vessel vessel, CelestialBody originBody, double vInf)
        {
            double muOrigin = originBody.gravParameter;
            double rPark = vessel.orbit.semiMajorAxis;
            double vCircPark = Math.Sqrt(muOrigin / rPark);
            double vHyperbolicAtPark = Math.Sqrt(vInf * vInf + 2.0 * muOrigin / rPark);
            return vHyperbolicAtPark - vCircPark;
        }

        // dv from hyperbolic arrival speed vInf down into a circular
        // parking orbit at destinationBody.Radius + 100km (placeholder
        // altitude - not chosen by the player yet).
        double VInfToCaptureDv(CelestialBody destinationBody, double vInf)
        {
            double muDest = destinationBody.gravParameter;
            double rCapture = destinationBody.Radius + 100000.0;
            double vCircCapture = Math.Sqrt(muDest / rCapture);
            double vHyperbolicAtCapture = Math.Sqrt(vInf * vInf + 2.0 * muDest / rCapture);
            return vHyperbolicAtCapture - vCircCapture;
        }

        // Universal-variable Lambert solver (short-way, single-revolution
        // branch). Solves for the two transfer-orbit velocity vectors that
        // connect r1vec to r2vec in time tof. Finds the root of the
        // Stumpff-function time equation by scanning the valid z-range for
        // a sign change and bisecting, rather than a closed-form Newton
        // derivative - slower, but nothing to get subtly wrong in the
        // derivative algebra. Returns false for a near-180-degree transfer
        // angle (singular for this method) or if no root is found in the
        // scanned range, rather than guessing.
        bool SolveLambert(Vector3d r1vec, Vector3d r2vec, double tof, double mu, out Vector3d v1, out Vector3d v2)
        {
            v1 = Vector3d.zero;
            v2 = Vector3d.zero;

            double r1 = r1vec.magnitude;
            double r2 = r2vec.magnitude;

            double crossZ = r1vec.x * r2vec.y - r1vec.y * r2vec.x;
            double cosDnu = Vector3d.Dot(r1vec, r2vec) / (r1 * r2);
            cosDnu = Math.Max(-1.0, Math.Min(1.0, cosDnu));
            double dnu = Math.Acos(cosDnu);
            if (crossZ < 0)
                dnu = 2.0 * Math.PI - dnu;

            double A = Math.Sin(dnu) * Math.Sqrt(r1 * r2 / (1.0 - Math.Cos(dnu)));
            if (Math.Abs(A) < 1e-6)
                return false; // near-180-degree transfer angle - singular for this method

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
                    bracketLoZ = prevZ; bracketLoF = prevF;
                    bracketHiZ = z;
                    haveBracket = true;
                    break;
                }

                prevZ = z; prevF = Fz; havePrev = true;
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
                    zLow = zMid; fLow = fMid;
                }
                else
                {
                    zHigh = zMid;
                }
            }

            double zFinal = (zLow + zHigh) / 2.0;
            if (!LambertY(zFinal, r1, r2, A, out double yFinal) || yFinal < 0)
                return false;

            double f = 1.0 - yFinal / r1;
            double g = A * Math.Sqrt(yFinal / mu);
            double gDot = 1.0 - yFinal / r2;

            v1 = (r2vec - f * r1vec) / g;
            v2 = (gDot * r2vec - r1vec) / g;
            return true;
        }

        bool LambertY(double z, double r1, double r2, double A, out double y)
        {
            double Cz = StumpffC(z);
            double Sz = StumpffS(z);
            y = r1 + r2 + A * (z * Sz - 1.0) / Math.Sqrt(Cz);
            return true;
        }

        bool LambertF(double z, double r1, double r2, double A, double mu, double tof, out double F)
        {
            F = 0;
            if (!LambertY(z, r1, r2, A, out double y) || y < 0)
                return false;

            double Cz = StumpffC(z);
            double Sz = StumpffS(z);
            F = Math.Pow(y / Cz, 1.5) * Sz + A * Math.Sqrt(y) - Math.Sqrt(mu) * tof;
            return true;
        }

        double StumpffC(double z)
        {
            if (z > 1e-6)
                return (1.0 - Math.Cos(Math.Sqrt(z))) / z;
            if (z < -1e-6)
                return (Math.Cosh(Math.Sqrt(-z)) - 1.0) / (-z);
            return 0.5 - z / 24.0 + z * z / 720.0;
        }

        double StumpffS(double z)
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

        // STILL UNCONFIRMED after actually checking: every other API call
        // in this file that was flagged UNVERIFIED got checked against the
        // real KSP API docs and either confirmed or fixed (see git history/
        // comments above). This one didn't - the old fan-maintained API
        // doc site used for those checks predates stock KSP's dV readout
        // (added ~1.11), and the GitHub source that would confirm the
        // exact field name (kOS's own delta-v reader, PR #2719) couldn't be
        // fetched from here. vessel.VesselDeltaV.TotalDeltaVActual is a
        // reasonable guess at the real name, not a confirmed one - check it
        // against Assembly-CSharp (e.g. with ILSpy/dnSpy) or that kOS PR
        // before trusting it.
        double GetVesselDeltaV(Vessel vessel)
        {
            if (vessel.VesselDeltaV == null)
                return 0.0;

            return vessel.VesselDeltaV.TotalDeltaVActual;
        }

    }


}