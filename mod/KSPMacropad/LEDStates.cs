namespace KSPMacropad
{
    public static class LEDStates
    {
        // key 1 - launch sequence (0x00)
        public const byte LAUNCH_IDLE = 0x00;
        public const byte LAUNCH_EXECUTING = 0x01;
        public const byte LAUNCH_COMPLETE = 0x02;

        // key 2 - landing prep (0x01)
        public const byte LANDING_IDLE = 0x00;
        public const byte LANDING_CONFIGURING = 0x01;
        public const byte LANDING_COMPLETE = 0x02;

        // key 3 - circularize setup (0x02)
        public const byte CIRC_IDLE = 0x00;
        public const byte CIRC_UNAVAILABLE = 0x01;
        public const byte CIRC_CALCULATING = 0x02;
        public const byte CIRC_WARPING = 0x03;
        public const byte CIRC_COMPLETE = 0x04;

        // key 4 - docking prep (0x03)
        public const byte DOCK_IDLE = 0x00;
        public const byte DOCK_ACTIVE = 0x01;
        public const byte DOCK_TARGET_ACQUIRED = 0x02;
        public const byte DOCK_READY = 0x03;

        // key 5 - deorbit burn (0x04)
        public const byte DEORBIT_IDLE = 0x00;
        public const byte DEORBIT_UNAVAILABLE = 0x01;
        public const byte DEORBIT_PLANNED = 0x02;
        public const byte DEORBIT_WARPING = 0x03;
        public const byte DEORBIT_BURNING = 0x04;

        // key 6 - time accel to next event (0x05)
        public const byte TIMEACCEL_IDLE = 0x00;
        public const byte TIMEACCEL_UNAVAILABLE = 0x01;
        public const byte TIMEACCEL_WARPING = 0x02;

        // key 7 - transmit science (0x06)
        public const byte SCIENCE_IDLE = 0x00;
        public const byte SCIENCE_UNAVAILABLE = 0x01;
        public const byte SCIENCE_TRANSMITTING = 0x02;
        public const byte SCIENCE_COMPLETE = 0x03;

        // key 8 - precision input toggle (0x07)
        // none - handled onboard by firmware

        // key 9 - auto gravity turn (0x08)
        public const byte AGT_IDLE = 0x00;
        public const byte AGT_ACTIVE = 0x01;

        // key 10 - intercept calc (0x09)
        public const byte INTERCEPT_IDLE = 0x00;
        public const byte INTERCEPT_CALCULATING = 0x01;
        public const byte INTERCEPT_SOLUTION = 0x02;
        public const byte INTERCEPT_INSUFFICIENT_DV = 0x03;

        // key 11 - suicide burn arm (0x0a)
        public const byte SUICIDEBURN_IDLE = 0x00;
        public const byte SUICIDEBURN_ARMED = 0x01;
        public const byte SUICIDEBURN_IMMINENT = 0x02;
        public const byte SUICIDEBURN_BURNING = 0x03;

        // key 12 - orbit sync (0x0b)
        public const byte ORBSYNC_IDLE = 0x00;
        public const byte ORBSYNC_CALCULATING = 0x01;
        public const byte ORBSYNC_EXECUTING = 0x02;
        public const byte ORBSYNC_COMPLETE = 0x03;

        // key 13 - rendezvous prep (0x0c)
        public const byte RENDEZVOUS_IDLE = 0x00;
        public const byte RENDEZVOUS_ACTIVE = 0x01;
        public const byte RENDEZVOUS_IN_RANGE = 0x02;

        // key 14 - resource monitor (0x0d)
        // idle = monitor not active, nominal = active and resources fine
        public const byte RESOURCE_IDLE = 0x00;
        public const byte RESOURCE_NOMINAL = 0x01;
        public const byte RESOURCE_LOW = 0x02;
        public const byte RESOURCE_VERYLOW = 0x03;
        public const byte RESOURCE_CRITICAL = 0x04;
        public const byte RESOURCE_DEPLETING = 0x05;

        // key 15 - auxiliary mode toggle (0x0e)
        // none - handled onboard by firmware

        // key 16 - autopilot (0x0f)
        public const byte AUTOPILOT_IDLE = 0x00;
        public const byte AUTOPILOT_IMPOSSIBLE = 0x01;
        public const byte AUTOPILOT_READY = 0x02;
        public const byte AUTOPILOT_PLANNING = 0x03;
        public const byte AUTOPILOT_EXECUTING = 0x04;

        // underglow (0x10-0x13)
        public const byte UNDERGLOW_IDLE = 0x00;
        public const byte UNDERGLOW_CONNECTED_NOVESSEL = 0x01;
        public const byte UNDERGLOW_LAUNCHPAD = 0x02;
        public const byte UNDERGLOW_ASCENT = 0x03;
        public const byte UNDERGLOW_STABLE_ORBIT = 0x04;
        public const byte UNDERGLOW_TIMEWARP = 0x05;
        public const byte UNDERGLOW_MANEUVER_IMMINENT = 0x06;
        public const byte UNDERGLOW_LANDING = 0x07;
        public const byte UNDERGLOW_AUTOPILOT = 0x08;
        public const byte UNDERGLOW_CRITICAL_RESOURCE = 0x09;
        public const byte UNDERGLOW_COMMS_LOST = 0x0A;
    }
}