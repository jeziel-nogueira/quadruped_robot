#!/usr/bin/env python3
"""
Servo Calibration Tool for Quadruped Robot
===========================================

Run this script on the Raspberry Pi to:
  1. Test each servo individually (sweep, center, manual angle)
  2. Adjust trim offsets until the robot stands straight
  3. Find the correct tick_min / tick_center / tick_max for your servos
  4. Set inversion for mirrored servos
  5. Save calibration to config.json

Usage:
  python -m robot.tools.calibrate_servos          # Interactive menu
  python -m robot.tools.calibrate_servos --sweep   # Quick sweep test all servos
  python -m robot.tools.calibrate_servos --center  # Center all servos
"""

import os
import sys
import json
import time
import argparse
import math

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from robot.hardware.servo_driver import (
    PCA9685ServoDriver,
    ServoConfig,
    build_default_servo_configs,
    JOINT_NAMES,
    DEFAULT_TICK_MIN,
    DEFAULT_TICK_CENTER,
    DEFAULT_TICK_MAX,
)

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config.json")


def load_config():
    """Load the full config.json."""
    try:
        with open(CONFIG_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"[!] config.json not found at {CONFIG_PATH}, using defaults.")
        return {}


def save_config(config_data):
    """Save config.json."""
    with open(CONFIG_PATH, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"[✓] Config saved to {CONFIG_PATH}")


def create_driver(config_data):
    """Create and connect servo driver with config."""
    servos_data = config_data.get("servos", [])
    if servos_data:
        servo_configs = [ServoConfig.from_dict(s) for s in servos_data]
    else:
        servo_configs = build_default_servo_configs()

    driver = PCA9685ServoDriver(servo_configs=servo_configs)
    if not driver.connect():
        print("\n[!] PCA9685 not detected! Make sure:")
        print("    - You're running on the Raspberry Pi")
        print("    - I2C is enabled (sudo raspi-config → Interface Options → I2C)")
        print("    - The PCA9685 board is wired to SDA/SCL")
        print("    - Adafruit_PCA9685 is installed (pip install Adafruit-PCA9685)")
        return None
    return driver


def print_servo_table(driver):
    """Print a table of current servo configs."""
    print("\n┌─────┬────────────────────┬─────────┬─────────┬──────────┬──────────┬──────────┬──────────────┐")
    print("│ Idx │ Name               │ Channel │ Tick Min│ Tick Ctr │ Tick Max │ Inverted │ Trim (°)     │")
    print("├─────┼────────────────────┼─────────┼─────────┼──────────┼──────────┼──────────┼──────────────┤")
    for i, cfg in enumerate(driver.servo_configs):
        inv_mark = "  ✓" if cfg.inverted else "  ✗"
        print(f"│ {i:>3} │ {cfg.name:<18} │   {cfg.channel:>3}   │  {cfg.tick_min:>5}  │  {cfg.tick_center:>5}   │  {cfg.tick_max:>5}   │  {inv_mark:>4}    │  {cfg.trim_deg:>+8.1f}    │")
    print("└─────┴────────────────────┴─────────┴─────────┴──────────┴──────────┴──────────┴──────────────┘")


def cmd_center_all(driver):
    """Center all servos to 90°."""
    print("\n[→] Moving all servos to center (90°)...")
    driver.center_all()
    print("[✓] All servos at center position.")


def cmd_detach_all(driver):
    """Detach (disable) all servos."""
    print("\n[→] Detaching all servos (PWM off)...")
    driver.detach_all()
    print("[✓] All servos detached.")


def cmd_sweep_all(driver):
    """Sweep all servos from 0° → 180° → 90° one at a time."""
    print("\n[→] Sweeping all servos sequentially (0° → 90° → 180° → 90°)...")
    print("    Press Ctrl+C to abort.\n")
    try:
        for i, cfg in enumerate(driver.servo_configs):
            print(f"    Servo {i} ({cfg.name}) channel {cfg.channel}...", end="", flush=True)

            # Move to 0°
            driver.set_angle_deg(i, 0)
            time.sleep(0.5)

            # Sweep to 180° in steps
            for deg in range(0, 181, 5):
                driver.set_angle_deg(i, deg)
                time.sleep(0.015)

            # Sweep back to 0°
            for deg in range(180, -1, -5):
                driver.set_angle_deg(i, deg)
                time.sleep(0.015)

            # Park at center
            driver.set_angle_deg(i, 90)
            time.sleep(0.2)

            print(" ✓")
    except KeyboardInterrupt:
        print("\n[!] Sweep aborted by user.")
        driver.center_all()


def cmd_test_single(driver):
    """Interactively test a single servo."""
    print_servo_table(driver)
    try:
        idx = int(input("\nEnter servo index (0-11): ").strip())
        if idx < 0 or idx > 11:
            print("[!] Invalid index.")
            return
    except (ValueError, EOFError):
        return

    cfg = driver.servo_configs[idx]
    print(f"\n[→] Testing servo {idx}: {cfg.name} (channel {cfg.channel})")
    print("    Commands:")
    print("      <number>   → Move to that angle in degrees (0-180)")
    print("      c          → Move to center (90°)")
    print("      s          → Sweep 0→180→0")
    print("      t <value>  → Set trim offset in degrees (e.g. 't 3.5' or 't -2')")
    print("      i          → Toggle inversion")
    print("      r          → Set raw PWM ticks directly")
    print("      q          → Back to main menu\n")

    while True:
        try:
            cmd = input(f"  [{cfg.name}]> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "q":
            break
        elif cmd == "c":
            driver.set_angle_deg(idx, 90)
            print(f"    → Moved to 90° (center)")
        elif cmd == "s":
            print("    → Sweeping...", end="", flush=True)
            for deg in range(0, 181, 3):
                driver.set_angle_deg(idx, deg)
                time.sleep(0.015)
            for deg in range(180, -1, -3):
                driver.set_angle_deg(idx, deg)
                time.sleep(0.015)
            driver.set_angle_deg(idx, 90)
            print(" done")
        elif cmd == "i":
            cfg.inverted = not cfg.inverted
            print(f"    → Inversion {'ON' if cfg.inverted else 'OFF'}")
        elif cmd.startswith("t "):
            try:
                val = float(cmd[2:])
                cfg.trim_deg = val
                print(f"    → Trim set to {val:+.1f}°")
                driver.set_angle_deg(idx, 90)
                print(f"    → Moved to 90° with trim applied")
            except ValueError:
                print("    [!] Invalid trim value")
        elif cmd == "r":
            try:
                ticks = int(input("    Enter raw ticks (0-4095): ").strip())
                driver.set_raw_ticks(cfg.channel, ticks)
                print(f"    → Raw ticks set to {ticks}")
            except (ValueError, EOFError):
                pass
        else:
            try:
                deg = float(cmd)
                if 0 <= deg <= 180:
                    driver.set_angle_deg(idx, deg)
                    print(f"    → Moved to {deg:.1f}°")
                else:
                    print("    [!] Angle must be 0-180")
            except ValueError:
                print("    [!] Unknown command")


def cmd_find_limits(driver):
    """Interactive helper to find tick_min and tick_max for a servo."""
    print_servo_table(driver)
    try:
        idx = int(input("\nEnter servo index (0-11): ").strip())
        if idx < 0 or idx > 11:
            print("[!] Invalid index.")
            return
    except (ValueError, EOFError):
        return

    cfg = driver.servo_configs[idx]
    print(f"\n[→] Finding limits for servo {idx}: {cfg.name} (channel {cfg.channel})")
    print("    This will send raw ticks to find the 0° and 180° positions.")
    print("    Commands:")
    print("      <number>   → Send raw ticks")
    print("      +          → Increase by 5 ticks")
    print("      -          → Decrease by 5 ticks")
    print("      ++         → Increase by 20 ticks")
    print("      --         → Decrease by 20 ticks")
    print("      min        → Save current ticks as tick_min (0°)")
    print("      ctr        → Save current ticks as tick_center (90°)")
    print("      max        → Save current ticks as tick_max (180°)")
    print("      q          → Done\n")

    current_ticks = cfg.tick_center

    driver.set_raw_ticks(cfg.channel, current_ticks)
    print(f"    Starting at {current_ticks} ticks")

    while True:
        try:
            cmd = input(f"  [{cfg.name} ticks={current_ticks}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "q":
            break
        elif cmd == "+":
            current_ticks += 5
        elif cmd == "-":
            current_ticks -= 5
        elif cmd == "++":
            current_ticks += 20
        elif cmd == "--":
            current_ticks -= 20
        elif cmd == "min":
            cfg.tick_min = current_ticks
            print(f"    → tick_min saved: {current_ticks}")
            continue
        elif cmd == "ctr":
            cfg.tick_center = current_ticks
            print(f"    → tick_center saved: {current_ticks}")
            continue
        elif cmd == "max":
            cfg.tick_max = current_ticks
            print(f"    → tick_max saved: {current_ticks}")
            continue
        else:
            try:
                current_ticks = int(cmd)
            except ValueError:
                print("    [!] Unknown command")
                continue

        current_ticks = max(0, min(4095, current_ticks))
        driver.set_raw_ticks(cfg.channel, current_ticks)
        print(f"    → Ticks: {current_ticks}")


def cmd_stand_pose(driver):
    """Move all servos to a standing pose for quick visual check."""
    print("\n[→] Moving to standing pose...")
    print("    All hip_yaw = 90°, hip_pitch = 45°, knee = 135°")

    for i in range(12):
        name = driver.servo_configs[i].name
        if "hip_yaw" in name:
            driver.set_angle_deg(i, 90)    # Neutral
        elif "hip_pitch" in name:
            driver.set_angle_deg(i, 45)    # Leaning forward
        elif "knee" in name:
            driver.set_angle_deg(i, 135)   # Bent backward
        time.sleep(0.05)

    print("[✓] Standing pose set. Adjust trims if legs aren't symmetrical.")


def cmd_save(driver, config_data):
    """Save current servo calibration to config.json."""
    config_data["servos"] = [cfg.to_dict() for cfg in driver.servo_configs]
    save_config(config_data)


def interactive_menu(driver, config_data):
    """Main interactive menu."""
    print("\n" + "=" * 60)
    print("  🐕 Quadruped Robot — Servo Calibration Tool")
    print("=" * 60)

    while True:
        print("\n  [1] Show servo config table")
        print("  [2] Center ALL servos (90°)")
        print("  [3] Sweep ALL servos (0° → 180° → 0°)")
        print("  [4] Test single servo (move to angles)")
        print("  [5] Find pulse limits (raw ticks)")
        print("  [6] Standing pose")
        print("  [7] Detach ALL servos (PWM off)")
        print("  [8] Save calibration to config.json")
        print("  [0] Exit\n")

        try:
            choice = input("  Choose> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "1":
            print_servo_table(driver)
        elif choice == "2":
            cmd_center_all(driver)
        elif choice == "3":
            cmd_sweep_all(driver)
        elif choice == "4":
            cmd_test_single(driver)
        elif choice == "5":
            cmd_find_limits(driver)
        elif choice == "6":
            cmd_stand_pose(driver)
        elif choice == "7":
            cmd_detach_all(driver)
        elif choice == "8":
            cmd_save(driver, config_data)
        elif choice == "0":
            cmd_detach_all(driver)
            break
        else:
            print("  [!] Invalid choice")

    print("\n[✓] Calibration tool finished.")


def main():
    parser = argparse.ArgumentParser(description="Servo Calibration Tool for Quadruped Robot")
    parser.add_argument("--sweep", action="store_true", help="Quick sweep all servos and exit")
    parser.add_argument("--center", action="store_true", help="Center all servos and exit")
    parser.add_argument("--stand", action="store_true", help="Move to standing pose and exit")
    args = parser.parse_args()

    config_data = load_config()
    driver = create_driver(config_data)
    if driver is None:
        print("\n[!] Cannot proceed without PCA9685 hardware.")
        print("    Re-run this script on the Raspberry Pi with the PCA9685 board connected.")
        sys.exit(1)

    if args.sweep:
        cmd_sweep_all(driver)
        cmd_detach_all(driver)
    elif args.center:
        cmd_center_all(driver)
    elif args.stand:
        cmd_stand_pose(driver)
    else:
        interactive_menu(driver, config_data)


if __name__ == "__main__":
    main()
