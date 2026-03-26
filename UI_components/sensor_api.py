"""
Sensor API - Unified entry point for sensor management.

Manages multiple named sensors, each with its own transport.
Provides simple read operations with typed accessors.

Usage:
    from sensor_api import SensorAPI
    from sensors.imu_i2c import IMUI2CTransport

    api = SensorAPI()

    imu = IMUI2CTransport()
    imu.connect()
    api.add_sensor('imu_hip', imu)

    reading = api.read('imu_hip')
    print(reading.imu.euler)  # (roll, pitch, yaw)

    data = api.get_imu('imu_hip')
    print(data.temperature)
"""
from __future__ import annotations

from typing import Dict, List, Optional

from sensor_types import IMUData, SensorReading
from sensor_transport import SensorTransport


class SensorAPI:
    """Unified sensor management interface."""

    def __init__(self):
        self._sensors: Dict[str, SensorTransport] = {}

    # ---- Sensor management ----

    def add_sensor(self, name: str, transport: SensorTransport):
        """Register a sensor with a unique name.

        Args:
            name: Unique identifier (e.g. 'imu_hip', 'imu_back').
            transport: Connected SensorTransport instance.
        """
        self._sensors[name] = transport

    def remove_sensor(self, name: str):
        """Remove and disconnect a sensor."""
        transport = self._sensors.pop(name, None)
        if transport:
            transport.disconnect()

    def get_transport(self, name: str) -> Optional[SensorTransport]:
        """Get the transport for a named sensor."""
        return self._sensors.get(name)

    # ---- Read operations ----

    def read(self, name: str) -> SensorReading:
        """Read one sample from a named sensor.

        Returns:
            SensorReading, or a failure reading if sensor not found.
        """
        transport = self._sensors.get(name)
        if not transport:
            return SensorReading(
                sensor_type="unknown",
                imu=IMUData.fail(f"Sensor '{name}' not registered"))
        return transport.read()

    def read_all(self) -> Dict[str, SensorReading]:
        """Read one sample from every registered sensor.

        Returns:
            Dict mapping sensor name to SensorReading.
        """
        return {name: transport.read()
                for name, transport in self._sensors.items()}

    def get_imu(self, name: str) -> Optional[IMUData]:
        """Convenience: read and return IMUData directly.

        Returns:
            IMUData or None if sensor not found or not an IMU.
        """
        reading = self.read(name)
        return reading.imu

    # ---- Batch operations ----

    def read_batch(self, name: str, n_samples: int) -> List[SensorReading]:
        """Read multiple samples from a named sensor."""
        transport = self._sensors.get(name)
        if not transport:
            return []
        return transport.read_batch(n_samples)

    # ---- Info ----

    @property
    def connected_sensors(self) -> List[str]:
        """List names of all connected sensors."""
        return [name for name, t in self._sensors.items() if t.is_connected]

    @property
    def all_sensors(self) -> List[str]:
        """List names of all registered sensors."""
        return list(self._sensors.keys())

    def get_info(self) -> dict:
        """Return configuration summary."""
        return {
            name: {
                'type': t.sensor_type,
                'transport': t.transport_name,
                'connected': t.is_connected,
                'sample_rate': t.sample_rate,
            }
            for name, t in self._sensors.items()
        }

    # ---- Lifecycle ----

    def disconnect_all(self):
        """Disconnect all sensors."""
        for transport in self._sensors.values():
            transport.disconnect()

    def shutdown(self):
        """Disconnect all sensors and clear registry."""
        self.disconnect_all()
        self._sensors.clear()
