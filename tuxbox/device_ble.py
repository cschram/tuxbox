#!/usr/bin/env python3
"""TuxBox BLE Driver - Linux Input Device

Implements Bluetooth Low Energy (BLE) transport for TourBox controllers.
Uses the reverse-engineered BLE protocol with evdev virtual input device.
"""

import sys
import os
import asyncio
import logging
from typing import Optional, Tuple

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from dbus_fast.aio import MessageBus
from dbus_fast import BusType, Message
from evdev import UInput

from .device_base import TuxBoxBase
from .config_loader import load_profiles
from .window_monitor import WindowMonitor
from .haptic import build_config_commands, HapticConfig

logger = logging.getLogger(__name__)

# BLE Configuration
VENDOR_SERVICE = "0000fff0-0000-1000-8000-00805f9b34fb"
NOTIFY_CHAR = "0000fff1-0000-1000-8000-00805f9b34fb"  # Button data notifications
WRITE_CHAR = "0000fff2-0000-1000-8000-00805f9b34fb"   # Commands to device

# Unlock sequence discovered from Windows BLE capture
UNLOCK_COMMAND = bytes.fromhex("5500078894001afe")

# Note: CONFIG_COMMANDS are now built dynamically by build_config_commands()
# from haptic.py to support per-profile haptic settings

# Device name prefix for scanning
TOURBOX_NAME_PREFIX = "TourBox"

# How long to wait for BlueZ to resolve a connected device's GATT services.
# Kept separate from the enumerate/scan timeout so the resolve wait doesn't
# inherit the full discovery budget.
SERVICE_RESOLVE_TIMEOUT = 5.0


async def _wait_services_resolved(bus: MessageBus, path: str, timeout: float) -> bool:
    """Poll a device's BlueZ ``ServicesResolved`` property until it becomes true.

    BlueZ resolves a device's GATT database asynchronously after the link comes
    up. If Bleak attaches before this completes, its connect step races GATT
    discovery against the link and can fail with "failed to discover services,
    device disconnected". Waiting here lets BlueZ finish first; we never touch
    the connection itself.

    Args:
        bus: An already-connected system MessageBus.
        path: The BlueZ D-Bus object path of the device.
        timeout: Maximum time to wait, in seconds.

    Returns:
        True if services resolved within the timeout, False otherwise.
    """
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        msg = Message(
            destination="org.bluez",
            path=path,
            interface="org.freedesktop.DBus.Properties",
            member="Get",
            signature="ss",
            body=["org.bluez.Device1", "ServicesResolved"],
        )
        try:
            reply = await bus.call(msg)
            if reply and reply.body and reply.body[0].value:
                return True
        except Exception:
            # Device may have vanished from BlueZ (disconnected/removed).
            return False
        await asyncio.sleep(0.3)
    return False


async def find_connected_tourbox(timeout: float = 10.0) -> Tuple[Optional[BLEDevice], bool]:
    """Check BlueZ for an already-connected TourBox device.

    BleakScanner cannot discover devices that are already connected to BlueZ.
    Rather than disconnecting and rescanning (which triggers BlueZ to immediately
    reconnect, causing a loop), we instead use the existing connection directly.

    Before returning, we wait for BlueZ to finish GATT service discovery
    (``ServicesResolved``). Attaching mid-discovery makes Bleak's connect race
    the link and fail; once services are resolved Bleak's discovery step returns
    immediately. BlueZ keeps control of the connection throughout.

    Constructs a BLEDevice with the BlueZ D-Bus path in details so that
    BleakClient can attach to the existing connection without scanning.

    Returns:
        A tuple ``(device, connected)``. ``device`` is a BLEDevice for an
        already-connected, services-resolved TourBox, or None if none was usable.
        ``connected`` is True when a TourBox is currently connected to BlueZ even
        if its services have not resolved yet; in that case the caller should
        retry this check rather than fall back to scanning, since BleakScanner
        cannot see already-connected devices.
    """
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    try:
        msg = Message(
            destination="org.bluez",
            path="/",
            interface="org.freedesktop.DBus.ObjectManager",
            member="GetManagedObjects",
        )

        try:
            res = await asyncio.wait_for(bus.call(msg), timeout=timeout)
        except asyncio.TimeoutError:
            logger.warning("Timeout while enumerating bluetooth devices")
            return None, False
        except Exception:
            logger.warning("Error while enumerating bluetooth devices")
            return None, False

        connected_found = False
        for path, props in res.body[0].items():
            device_info = props.get('org.bluez.Device1')
            if not device_info:
                continue

            alias = device_info.get('Alias') or device_info.get('Name')
            name = alias.value if alias else None
            connected = device_info.get('Connected')
            connected = connected.value if connected else False

            if not connected or not name or not name.startswith(TOURBOX_NAME_PREFIX):
                continue

            # A TourBox is connected to BlueZ; a rescan can't improve on this
            # even if this particular entry turns out to be unusable below.
            connected_found = True

            # D-Bus path format: /org/bluez/hci0/dev_DD_6A_5F_61_0E_12
            address = path.split('/')[-1].replace('dev_', '').replace('_', ':')

            # Let BlueZ finish service discovery before Bleak attaches.
            resolved = device_info.get('ServicesResolved')
            resolved = resolved.value if resolved else False
            if not resolved:
                logger.info(f"Found {name}; waiting for BlueZ to resolve services...")
                resolved = await _wait_services_resolved(bus, path, SERVICE_RESOLVE_TIMEOUT)
                if not resolved:
                    logger.warning(
                        f"{name} connected but services not resolved within "
                        f"{SERVICE_RESOLVE_TIMEOUT}s"
                    )
                    # Another adapter may expose a usable entry; keep looking.
                    continue

            logger.info(f"Found already connected TourBox device: {name} at {address}")
            # Pass the D-Bus path in details so BleakClient uses the existing
            # BlueZ connection object directly rather than trying to scan for it.
            return BLEDevice(address, name, {"path": path}), True

        return None, connected_found
    finally:
        bus.disconnect()


async def scan_for_tuxbox(timeout: float = 10.0) -> Optional[BLEDevice]:
    """Find a TourBox device, checking for an existing BlueZ connection first.

    Checks whether a TourBox is already connected to BlueZ (e.g. auto-connected
    by a Bluetooth manager) and returns it directly. Falls back to a BLE scan
    only when no existing connection is found.

    Args:
        timeout: Scan timeout in seconds (default 10.0)

    Returns:
        BLEDevice if found, None otherwise.
    """
    device, connected = await find_connected_tourbox(timeout)
    if device:
        print(f"Found {device.name} at {device.address}")
        return device
    if connected:
        # A TourBox is connected but BlueZ hasn't resolved its services yet.
        # BleakScanner cannot see already-connected devices, so a scan would be
        # futile; return None and let the reconnect loop retry the BlueZ check.
        return None

    logger.info(f"Scanning for TourBox devices (timeout: {timeout}s)...")
    print("Scanning for TourBox devices...")

    found_device: Optional[BLEDevice] = None
    stop_event = asyncio.Event()

    def detection_callback(device: BLEDevice, adv_data):
        nonlocal found_device
        if device.name and device.name.startswith(TOURBOX_NAME_PREFIX):
            logger.info(f"Found {device.name} at {device.address}")
            print(f"Found {device.name} at {device.address}")
            found_device = device
            stop_event.set()

    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()

    try:
        await asyncio.wait_for(stop_event.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        pass
    finally:
        await scanner.stop()

    if not found_device:
        logger.warning("No TourBox device found during scan")
        print("No TourBox device found. Make sure your TourBox is powered on.")

    return found_device


class TuxBoxBLE(TuxBoxBase):
    """TuxBox BLE Driver

    Implements BLE transport using Bleak library. Inherits common functionality
    from TuxBoxBase including button processing, modifier state machine,
    profile management, and virtual input device handling.
    """

    def __init__(self, pidfile: Optional[str] = None, config_path: Optional[str] = None):
        """Initialize the BLE driver

        Args:
            pidfile: Path to PID file
            config_path: Path to configuration file
        """
        super().__init__(pidfile=pidfile, config_path=config_path)
        self.device: Optional[BLEDevice] = None
        self.client: Optional[BleakClient] = None
        self.disconnected = False
        self.reconnect_delay = 5.0  # Initial reconnection delay in seconds

    def disconnection_handler(self, client):
        """Handle disconnection from TourBox Elite"""
        # Bleak/bluez can invoke this callback multiple times for a single
        # failing session (e.g. during connect retries). Only react to the
        # first one so logs and modifier-reset don't fire repeatedly.
        if self.disconnected:
            return
        self.disconnected = True
        self.clear_modifier_state()
        logger.warning("TourBox Elite disconnected")
        print("\nTourBox Elite disconnected - waiting for reconnection...")

    def notification_handler(self, sender, data: bytearray):
        """Handle button/dial notifications from TourBox Elite

        Wraps the base class process_button_code method for BLE notifications.
        """
        for byte in data:
            self.process_button_code(bytearray([byte]))

    async def send_haptic_config(self):
        """Send haptic configuration to the device

        Called when profile switches to apply the new profile's haptic settings.
        """
        if not self.client or not self.client.is_connected:
            logger.warning("Cannot send haptic config - not connected")
            return

        haptic_config = None
        if self.current_profile and self.current_profile.haptic_config:
            haptic_config = self.current_profile.haptic_config
            logger.info(f"Sending haptic config for profile '{self.current_profile.name}': {haptic_config}")

        config_commands = build_config_commands(haptic_config)

        # Send configuration commands
        for cmd in config_commands:
            await self.client.write_gatt_char(WRITE_CHAR, cmd, response=False)
            await asyncio.sleep(0.01)

        logger.info("Haptic configuration sent")

    async def unlock_device(self):
        """Send unlock sequence to TourBox Elite

        This is required before the device will report button presses.
        Sequence discovered via Windows BLE traffic capture.
        """
        logger.info("Sending unlock command...")

        # Send unlock command
        await self.client.write_gatt_char(WRITE_CHAR, UNLOCK_COMMAND, response=False)
        await asyncio.sleep(0.1)

        logger.info("Sending configuration commands...")

        # Send haptic configuration for initial profile
        await self.send_haptic_config()

        logger.info("Device unlocked and configured")

    async def connect(self) -> bool:
        """Connect to the TourBox via BLE

        Note: This method exists to satisfy the abstract base class.
        The actual connection is handled by run_connection() which includes
        device scanning.

        Returns:
            True (connection is handled by run_connection)
        """
        return True

    async def disconnect(self):
        """Disconnect from the TourBox Elite"""
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(NOTIFY_CHAR)
            except Exception:
                pass
            await self.client.disconnect()

    async def run_connection(self) -> bool:
        """Run a single BLE connection session

        Returns:
            True if should retry connection, False if user requested exit
        """
        try:
            # Scan for TourBox device (or pick up existing BlueZ connection)
            self.device = await scan_for_tuxbox(timeout=10.0)
            if not self.device:
                logger.error("No TourBox device found")
                return True  # Retry (will scan again)

            self.device_name = self.device.name or "TourBox"
            logger.info(f"Connecting to {self.device_name} at {self.device.address}...")
            self.disconnected = False

            async with BleakClient(
                self.device,
                timeout=5.0,
                disconnected_callback=self.disconnection_handler
            ) as client:
                self.client = client
                logger.info("Connected to TourBox Elite")

                # Unlock device and send configuration
                await self.unlock_device()

                # Enable notifications
                logger.info("Enabling button notifications...")
                await client.start_notify(NOTIFY_CHAR, self.notification_handler)

                logger.info("TourBox Elite ready! Press buttons to generate input events.")
                print("TourBox Elite connected and ready!")
                dev_path = self.controller.device.path if self.controller.device else "unknown"
                print(f"Virtual input device: {dev_path}")

                if self.use_profiles:
                    print(f"Profile switching enabled - Current profile: {self.current_profile.name}")

                print("Press Ctrl+C to exit")

                # Reset reconnect delay on successful connection
                self.reconnect_delay = 5.0

                # Keep running until disconnected or killed
                while not self.killer.kill_now and not self.disconnected:
                    # Check if config reload was requested
                    if self.killer.reload_config:
                        self.reload_config_mappings()
                        self.killer.reload_config = False

                    await asyncio.sleep(0.5)

                # Check if user requested exit
                if self.killer.kill_now:
                    logger.info("Shutting down...")
                    await client.stop_notify(NOTIFY_CHAR)
                    return False  # Don't reconnect

                # Device disconnected, will reconnect
                return True

        except asyncio.TimeoutError:
            logger.error("Timed out establishing BLE connection to TourBox")
            print("Timed out connecting to TourBox - will retry")
            return True  # Retry
        except Exception as ex:
            logger.error(f"Connection error: {ex}")
            print(f"Connection error: {ex}")
            return True  # Retry

    async def start(self):
        """Start the TourBox BLE driver with automatic reconnection"""
        import pathlib

        # Load profiles from config
        self.profiles = load_profiles(self.config_path)

        if not self.profiles:
            logger.error("No profiles found in config file")
            print("Error: Config file must contain at least one [profile:default] section")
            print("")
            print("Your config file should use the profile format:")
            print("")
            print("[profile:default]")
            print("side = KEY_LEFTMETA")
            print("top = KEY_LEFTSHIFT")
            print("# ... (all buttons and rotary controls)")
            print("")
            print("See the default config for a complete example:")
            print("  cat tuxbox/default_mappings.conf")
            print("")
            print("Or see the config guide:")
            print("  https://github.com/your-repo/docs/CONFIG_GUIDE.md")
            sys.exit(1)

        self.use_profiles = True
        logger.info(f"Loaded {len(self.profiles)} profiles")

        # Find default profile
        default_profile = next((p for p in self.profiles if p.name == 'default'), None)
        if default_profile:
            self.current_profile = default_profile
            self.mapping = default_profile.mapping
            self.capabilities = default_profile.capabilities
            logger.info(f"Using default profile: {default_profile}")
        else:
            # Use first profile as default
            self.current_profile = self.profiles[0]
            self.mapping = self.profiles[0].mapping
            self.capabilities = self.profiles[0].capabilities
            logger.warning(f"No 'default' profile found, using '{self.profiles[0].name}' as fallback")

        # Initialize window monitor
        self.window_monitor = WindowMonitor()

        # Write PID file
        pid = str(os.getpid())
        p = pathlib.Path(self.pidfile)
        p.write_text(pid)

        # Create virtual input device (persists across reconnections)
        self.create_virtual_device()

        # Start window monitoring if using profiles
        monitor_task = None
        if self.use_profiles and self.window_monitor and self.window_monitor.compositor:
            logger.info("Starting window monitor for profile switching")
            monitor_task = asyncio.create_task(
                self.window_monitor.monitor_window_changes(self.on_window_change, interval=0.2)
            )

        # Connection loop with automatic reconnection
        try:
            while not self.killer.kill_now:
                # Check if config reload was requested
                if self.killer.reload_config:
                    self.reload_config_mappings()
                    self.killer.reload_config = False

                should_retry = await self.run_connection()

                if not should_retry:
                    break

                # Wait before reconnecting with exponential backoff
                if not self.killer.kill_now:
                    delay = min(self.reconnect_delay, 10.0)  # Cap at 10 seconds
                    logger.info(f"Attempting to reconnect in {delay} seconds...")
                    print(f"Reconnecting in {delay} seconds...")
                    await asyncio.sleep(delay)
                    self.reconnect_delay = min(self.reconnect_delay * 1.5, 10.0)
        except KeyboardInterrupt:
            pass
        finally:
            # Cancel monitor task if running
            if monitor_task:
                monitor_task.cancel()
                try:
                    await monitor_task
                except asyncio.CancelledError:
                    pass

            # Cleanup
            self.cleanup()
            logger.info("TuxBox BLE driver stopped")


def main():
    """Main entry point for BLE driver"""
    import argparse

    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='TuxBox BLE Driver')
    parser.add_argument('-c', '--config', help='Path to custom config file')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    # Enable debug logging if requested
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create and start driver (will scan for device)
    driver = TuxBoxBLE(config_path=args.config)

    try:
        asyncio.run(driver.start())
    except KeyboardInterrupt:
        print("\nExited by user")
    except Exception as ex:
        logger.error(f"Error: {ex}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
