using System;
using System.IO.Ports;
using UnityEngine;

namespace KSPMacropad
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPMacropad : MonoBehaviour
    {
        private SerialPort serial;

        void Start()
        {
            serial = new SerialPort("COM3", 115200);
            serial.Open();
        }

        void Update()
        {
            if (FlightGlobals.ActiveVessel == null) return;

            Vessel v = FlightGlobals.ActiveVessel;
        }

        void OnDestroy()
        {
            if (serial != null && serial.IsOpen)
                serial.Close();
        }
    }
}