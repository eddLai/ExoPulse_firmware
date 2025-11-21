#pragma once

/**
 * Output Mode Configuration
 * Shared between serial_commands.h and wifi_pairing.h
 */

// Output mode enumeration
enum OutputMode {
    MODE_SERIAL,  // Output to Serial only (default)
    MODE_WIFI,    // Output to WiFi TCP client only
    MODE_BOTH     // Output to both Serial and WiFi (debug mode)
};

// External reference to current output mode (defined in main.cpp)
extern volatile OutputMode currentOutputMode;
