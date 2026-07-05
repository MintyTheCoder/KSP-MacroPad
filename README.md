# KSP Macropad

A custom macropad built specifically for Kerbal Space Program, developed as part of Hack Club's Stardance Challenge.

16 macro keys + 2 rotary encoders on a custom PCB driven by a Seeed XIAO RP2040. The keys execute complex multi-step sequences like launch procedures, orbital maneuvers, and autonomous mission execution via a companion KSP mod. The mod communicates over USB serial, reads live game telemetry, and sends it to a companion desktop app for display.

The key matrix uses an ADC-based resistor ladder instead of a traditional GPIO matrix, a custom solution to a pin shortage on the RP2040.

## Branches
- `mod` — KSP C# plugin
- `design` — PCB schematic and case files