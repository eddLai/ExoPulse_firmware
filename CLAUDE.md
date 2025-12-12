# ExoPulse Firmware Project Notes

## MCP Server: Nordic Semiconductor Documentation

A Nordic Semiconductor MCP server is configured (`nordic-docs`) that provides access to nRF SDK documentation via Kapa.ai.

**How to use:**
- The MCP tool is named `nordic-docs`
- Use it to query Nordic documentation for nRF52840, nRF5 SDK, SoftDevice, BLE stack, USB CDC, DFU, etc.
- Example: Ask about nRF52840 USB CDC implementation, BLE advertising, GATT services, etc.

**Configuration location:** `~/.claude/settings.json`

## Project Structure

- `EMG/uMyo/` - uMyo EMG sensor firmware (nRF52832)
- `EMG/uMyo/nrf52840_dongle_receiver/` - USB dongle receiver firmware (nRF52840)
- Uses PlatformIO for building
- `launch.sh` - Unified flash and GUI portal

## Hardware

- **Sensor:** nRF52832-based uMyo EMG sensor
- **Receiver:** nRF52840 USB dongle (acts as USB CDC device)
- Communication: Proprietary 2.4GHz radio protocol between sensor and dongle
