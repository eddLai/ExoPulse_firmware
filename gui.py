#!/usr/bin/env python3
"""
ExoPulse GUI Entry Point

Launch user interface components for ExoPulse motor control and monitoring.
"""

import sys
import os
from pathlib import Path

# Add UI_components to path
UI_DIR = Path(__file__).parent / "UI_components"
sys.path.insert(0, str(UI_DIR))


def print_menu():
    """Display GUI selection menu"""
    print("=" * 70)
    print("  ExoPulse GUI Launcher")
    print("=" * 70)
    print("\nLow-Level Components (Basic Control):")
    print("  1. Motor Control          - Basic motor control GUI")
    print("  2. Serial Reader          - Simple serial data reader")
    print("\nHigh-Level Components (Advanced Monitoring):")
    print("  3. Motor Monitor          - Advanced monitoring with configurable plots")
    print("  4. Dual Motor Plotter     - Real-time dual motor visualization")
    print("  5. WiFi Monitor           - WiFi-based remote monitoring")
    print("  6. WiFi Dual Motor Plot   - WiFi dual motor plotter")
    print("  7. CAN Plotter            - CAN bus data visualization")
    print("  8. EMG Plotter            - EMG signal visualization")
    print("\n  0. Exit")
    print("=" * 70)


def launch_gui(choice):
    """Launch selected GUI component"""

    components = {
        '1': ('motor_control', 'Motor Control'),
        '2': ('serial_reader', 'Serial Reader'),
        '3': ('motor_monitor', 'Motor Monitor'),
        '4': ('dual_motor_plotter', 'Dual Motor Plotter'),
        '5': ('wifi_monitor', 'WiFi Monitor'),
        '6': ('wifi_dual_motor_plotter', 'WiFi Dual Motor Plotter'),
        '7': ('can_plotter', 'CAN Plotter'),
        '8': ('emg_plotter', 'EMG Plotter'),
    }

    if choice not in components:
        return False

    module_name, display_name = components[choice]
    script_path = UI_DIR / f"{module_name}.py"

    if not script_path.exists():
        print(f"\n✗ Error: {script_path} not found!")
        return False

    print(f"\n🚀 Launching {display_name}...")
    print(f"   Script: {script_path}")
    print("-" * 70)

    # Execute the script
    try:
        with open(script_path, 'r') as f:
            code = compile(f.read(), script_path, 'exec')
            exec(code, {'__name__': '__main__', '__file__': str(script_path)})
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\n✗ Error launching {display_name}: {e}")
        import traceback
        traceback.print_exc()

    return True


def main():
    """Main entry point"""

    # Check if UI_components directory exists
    if not UI_DIR.exists():
        print(f"✗ Error: UI_components directory not found at {UI_DIR}")
        print("  Make sure you're running this script from the project root.")
        sys.exit(1)

    # If command-line argument provided, launch directly
    if len(sys.argv) > 1:
        choice = sys.argv[1]
        if choice == 'help' or choice == '-h' or choice == '--help':
            print_menu()
            print("\nUsage:")
            print("  python3 gui.py [option]")
            print("\nExamples:")
            print("  python3 gui.py          # Interactive menu")
            print("  python3 gui.py 3        # Launch Motor Monitor directly")
            print("  python3 gui.py 4        # Launch Dual Motor Plotter directly")
            return

        if not launch_gui(choice):
            print(f"\n✗ Invalid option: {choice}")
            print("   Run 'python3 gui.py help' for available options")
            sys.exit(1)
        return

    # Interactive mode
    while True:
        print_menu()
        try:
            choice = input("\nSelect option (0-8): ").strip()

            if choice == '0':
                print("\n👋 Goodbye!")
                break

            if not launch_gui(choice):
                print(f"\n✗ Invalid option: {choice}")
                input("\nPress Enter to continue...")

        except KeyboardInterrupt:
            print("\n\n👋 Goodbye!")
            break
        except EOFError:
            print("\n\n👋 Goodbye!")
            break


if __name__ == '__main__':
    main()
