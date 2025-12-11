#!/bin/bash
# ============================================================================
# ExoPulse Unified Launch Script
# ============================================================================
# Usage: ./launch.sh [command]
#
# Commands:
#   gui          - Launch main ExoPulse GUI (default)
#   emg-gui      - Launch EMG/uMyo GUI
#   emg-plot     - Launch EMG real-time plotter
#   flash-exo    - Flash ESP32 Exo dongle firmware (MGv2)
#   flash-umyo   - Flash uMyo sensor firmware (nRF52810)
#   flash-dongle - Flash nRF52840 USB dongle receiver
#   flash-dk     - Flash nRF52840 DK receiver
#   flash-ble    - Flash ESP32 BLE scanner
#   build-all    - Build all firmware targets
#   help         - Show this help message
#
# Environment:
#   Python scripts use 'forward_sim' conda environment
#   Flash/build commands use 'base' conda environment
# ============================================================================

set -e

# Project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Conda environments
PYTHON_ENV="forward_sim"
FLASH_ENV="base"

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo -e "${CYAN}============================================================================${NC}"
    echo -e "${CYAN} $1${NC}"
    echo -e "${CYAN}============================================================================${NC}"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Activate conda environment
activate_env() {
    local env_name=$1
    print_info "Activating conda environment: $env_name"

    # Source conda
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/anaconda3/etc/profile.d/conda.sh"
    elif [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
        source "/opt/conda/etc/profile.d/conda.sh"
    else
        print_warning "Conda not found in standard locations, trying direct activation"
    fi

    conda activate "$env_name" 2>/dev/null || {
        print_warning "Failed to activate $env_name, using current environment"
    }
}

# Check if command exists
check_command() {
    if ! command -v "$1" &> /dev/null; then
        print_error "$1 is not installed or not in PATH"
        return 1
    fi
    return 0
}

# ============================================================================
# GUI Launch Functions
# ============================================================================

launch_main_gui() {
    print_header "Launching ExoPulse Main GUI"
    activate_env "$PYTHON_ENV"

    print_info "Starting gui.py..."
    python3 "$SCRIPT_DIR/gui.py"
}

launch_emg_gui() {
    print_header "Launching EMG/uMyo GUI"
    activate_env "$PYTHON_ENV"

    local gui_script="$SCRIPT_DIR/EMG/nRF_DK/receiver/umyo_plotter_gui.py"
    if [ -f "$gui_script" ]; then
        print_info "Starting umyo_plotter_gui.py..."
        python3 "$gui_script"
    else
        print_error "EMG GUI not found at: $gui_script"
        exit 1
    fi
}

launch_emg_plot() {
    print_header "Launching EMG Real-time Plotter"
    activate_env "$PYTHON_ENV"

    local plot_script="$SCRIPT_DIR/EMG/nRF_DK/receiver/plot_umyo.py"
    if [ -f "$plot_script" ]; then
        print_info "Starting plot_umyo.py..."
        print_info "Usage: Specify serial port with -p flag (e.g., -p /dev/ttyACM0)"
        python3 "$plot_script" "$@"
    else
        print_error "EMG plotter not found at: $plot_script"
        exit 1
    fi
}

# ============================================================================
# Flash Functions
# ============================================================================

flash_exo_dongle() {
    print_header "Flashing ESP32 Exo Dongle (MGv2)"
    activate_env "$FLASH_ENV"

    check_command "pio" || {
        print_error "PlatformIO not found. Install with: pip install platformio"
        exit 1
    }

    print_info "Building and uploading MGv2 firmware..."
    print_info "Environment: env:MGv2"

    cd "$SCRIPT_DIR"
    pio run -e MGv2 -t upload

    print_success "ESP32 Exo dongle flashed successfully"
}

flash_umyo_sensor() {
    print_header "Flashing uMyo Sensor Firmware (nRF52810)"
    activate_env "$FLASH_ENV"

    check_command "arm-none-eabi-gcc" || {
        print_error "ARM toolchain not found. Install with: sudo apt install gcc-arm-none-eabi"
        exit 1
    }

    local umyo_dir="$SCRIPT_DIR/EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update"

    if [ ! -d "$umyo_dir" ]; then
        print_error "uMyo firmware directory not found: $umyo_dir"
        exit 1
    fi

    print_info "Building uMyo firmware..."
    cd "$umyo_dir"
    make clean && make

    print_info "Flashing via OpenOCD + ST-LINK..."
    print_warning "Ensure ST-LINK/V2 is connected to uMyo"

    # Flash using OpenOCD
    openocd -f interface/stlink.cfg \
            -f target/nrf52.cfg \
            -c "program build/uMyo.hex verify reset exit" || {
        print_error "Flash failed. Check ST-LINK connection."
        exit 1
    }

    print_success "uMyo sensor flashed successfully"
}

flash_nrf_dongle() {
    print_header "Flashing nRF52840 USB Dongle Receiver"
    activate_env "$FLASH_ENV"

    check_command "arm-none-eabi-gcc" || {
        print_error "ARM toolchain not found. Install with: sudo apt install gcc-arm-none-eabi"
        exit 1
    }

    local dongle_dir="$SCRIPT_DIR/EMG/uMyo/nrf52840_dongle_receiver"

    if [ ! -d "$dongle_dir" ]; then
        print_error "Dongle receiver directory not found: $dongle_dir"
        exit 1
    fi

    print_info "Building nRF52840 dongle receiver..."
    cd "$dongle_dir"
    make clean && make

    print_info "Flashing via nrfutil DFU..."
    print_warning "Put dongle in bootloader mode (press reset while holding button)"

    # Generate DFU package and flash
    # Note: Adjust this based on your bootloader setup
    if command -v nrfutil &> /dev/null; then
        nrfutil pkg generate --hw-version 52 --sd-req 0x00 \
            --application build/nrf52840_dongle_receiver.hex \
            --application-version 1 build/dfu_package.zip

        print_info "Looking for DFU device..."
        nrfutil dfu usb-serial -pkg build/dfu_package.zip -p /dev/ttyACM0 || {
            print_warning "DFU flash failed. Try specifying correct port."
            print_info "Available ports:"
            ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial ports found"
        }
    else
        print_warning "nrfutil not found. Manual flash required."
        print_info "Hex file: $dongle_dir/build/nrf52840_dongle_receiver.hex"
    fi

    print_success "nRF52840 dongle flash completed"
}

flash_nrf_dk() {
    print_header "Flashing nRF52840 DK Receiver"
    activate_env "$FLASH_ENV"

    check_command "arm-none-eabi-gcc" || {
        print_error "ARM toolchain not found. Install with: sudo apt install gcc-arm-none-eabi"
        exit 1
    }

    check_command "JLinkExe" || {
        print_error "J-Link tools not found. Install from: https://www.segger.com/downloads/jlink/"
        exit 1
    }

    local dk_dir="$SCRIPT_DIR/EMG/nRF_DK/receiver"

    if [ ! -d "$dk_dir" ]; then
        print_error "DK receiver directory not found: $dk_dir"
        exit 1
    fi

    print_info "Building nRF52840 DK receiver..."
    cd "$dk_dir"
    make clean && make

    print_info "Flashing via J-Link..."
    make flash

    print_success "nRF52840 DK flashed successfully"
}

flash_ble_scanner() {
    print_header "Flashing ESP32 BLE Scanner"
    activate_env "$FLASH_ENV"

    check_command "pio" || {
        print_error "PlatformIO not found. Install with: pip install platformio"
        exit 1
    }

    local ble_dir="$SCRIPT_DIR/ble_scanner"

    if [ ! -d "$ble_dir" ]; then
        print_error "BLE scanner directory not found: $ble_dir"
        exit 1
    fi

    print_info "Building and uploading BLE scanner..."
    cd "$ble_dir"
    pio run -t upload

    print_success "ESP32 BLE scanner flashed successfully"
}

# ============================================================================
# Build Functions
# ============================================================================

build_all() {
    print_header "Building All Firmware Targets"
    activate_env "$FLASH_ENV"

    local failed=0

    # Build ESP32 MGv2
    print_info "Building ESP32 MGv2..."
    cd "$SCRIPT_DIR"
    pio run -e MGv2 || { print_error "MGv2 build failed"; failed=1; }

    # Build ESP32 BLE Scanner
    print_info "Building ESP32 BLE Scanner..."
    cd "$SCRIPT_DIR/ble_scanner"
    pio run || { print_error "BLE Scanner build failed"; failed=1; }

    # Build uMyo firmware
    print_info "Building uMyo firmware..."
    cd "$SCRIPT_DIR/EMG/uMyo/uMyo_fw_v3_1_April_2025_RGB_update"
    make clean && make || { print_error "uMyo build failed"; failed=1; }

    # Build nRF52840 DK receiver
    print_info "Building nRF52840 DK receiver..."
    cd "$SCRIPT_DIR/EMG/nRF_DK/receiver"
    make clean && make || { print_error "DK receiver build failed"; failed=1; }

    # Build nRF52840 dongle receiver
    print_info "Building nRF52840 dongle receiver..."
    cd "$SCRIPT_DIR/EMG/uMyo/nrf52840_dongle_receiver"
    make clean && make || { print_error "Dongle receiver build failed"; failed=1; }

    if [ $failed -eq 0 ]; then
        print_success "All firmware builds completed successfully"
    else
        print_error "Some builds failed"
        exit 1
    fi
}

# ============================================================================
# Help Function
# ============================================================================

show_help() {
    cat << 'EOF'
============================================================================
                    ExoPulse Unified Launch Script
============================================================================

USAGE:
    ./launch.sh [command] [options]

COMMANDS:
    gui              Launch main ExoPulse GUI (default)
    emg-gui          Launch EMG/uMyo PySide6 GUI
    emg-plot         Launch EMG real-time matplotlib plotter
                     Options: -p /dev/ttyACM0 (serial port)

    flash-exo        Flash ESP32 Exo dongle firmware (MGv2 via PlatformIO)
    flash-umyo       Flash uMyo sensor firmware (nRF52810 via OpenOCD+ST-LINK)
    flash-dongle     Flash nRF52840 USB dongle receiver (via nrfutil DFU)
    flash-dk         Flash nRF52840 DK receiver (via J-Link)
    flash-ble        Flash ESP32 BLE scanner (via PlatformIO)

    build-all        Build all firmware targets without flashing

    help             Show this help message

ENVIRONMENTS:
    Python scripts:  conda activate forward_sim
    Flash/build:     conda activate base

HARDWARE CONNECTIONS:
    ┌─────────────────┬──────────────────┬─────────────────────────┐
    │ Target          │ Programmer       │ Connection              │
    ├─────────────────┼──────────────────┼─────────────────────────┤
    │ ESP32 (Exo/BLE) │ USB-Serial       │ /dev/ttyUSB0            │
    │ uMyo (nRF52810) │ ST-LINK/V2       │ SWD pins                │
    │ nRF52840 DK     │ J-Link OB        │ USB (J-Link port)       │
    │ nRF52840 Dongle │ Built-in DFU     │ USB (bootloader mode)   │
    └─────────────────┴──────────────────┴─────────────────────────┘

EXAMPLES:
    ./launch.sh                    # Launch main GUI
    ./launch.sh gui                # Same as above
    ./launch.sh emg-gui            # Launch EMG GUI
    ./launch.sh emg-plot -p /dev/ttyACM0
    ./launch.sh flash-exo          # Flash ESP32 motor controller
    ./launch.sh flash-dk           # Flash nRF52840 DK receiver
    ./launch.sh build-all          # Build all without flashing

QUICK START:
    1. Flash ESP32 Exo dongle:     ./launch.sh flash-exo
    2. Flash nRF52840 DK:          ./launch.sh flash-dk
    3. Launch main GUI:            ./launch.sh gui
    4. Launch EMG GUI:             ./launch.sh emg-gui

============================================================================
EOF
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    local command="${1:-gui}"
    shift 2>/dev/null || true

    case "$command" in
        gui)
            launch_main_gui
            ;;
        emg-gui)
            launch_emg_gui
            ;;
        emg-plot)
            launch_emg_plot "$@"
            ;;
        flash-exo)
            flash_exo_dongle
            ;;
        flash-umyo)
            flash_umyo_sensor
            ;;
        flash-dongle)
            flash_nrf_dongle
            ;;
        flash-dk)
            flash_nrf_dk
            ;;
        flash-ble)
            flash_ble_scanner
            ;;
        build-all)
            build_all
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "Unknown command: $command"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
