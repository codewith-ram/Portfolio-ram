"""
Mavlink v2.0 communication handler for ArduPilot GCS
"""
import time
from pymavlink import mavutil
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class TelemetryData:
    """Container for telemetry data"""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0
    groundspeed: float = 0.0
    battery_voltage: float = 0.0
    battery_remaining: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    system_status: str = "UNKNOWN"
    timestamp: float = 0.0

class MavlinkHandler:
    """Handles Mavlink v2.0 communication with ArduPilot"""
    
    def __init__(self, connection_string: str = 'udpin:localhost:14550', 
                 baud: int = 57600, 
                 source_system: int = 255, 
                 source_component: int = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER):
        """
        Initialize Mavlink connection
        
        Args:
            connection_string: Connection string (e.g., 'udpin:localhost:14550' or '/dev/ttyUSB0')
            baud: Baud rate for serial connections
            source_system: System ID of the GCS
            source_component: Component ID of the GCS
        """
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.master = None
        self.telemetry = TelemetryData()
        self.connected = False
        self.heartbeat_timeout = 3  # seconds
        self.last_heartbeat = 0
        self.vehicle_type = None
        self.vehicle_autopilot = None
        self.vehicle_system = None
        self.vehicle_component = None
        self.mission_items = []
        self.parameters = {}
        
    def connect(self) -> bool:
        """Establish connection to the vehicle"""
        try:
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud,
                source_system=self.source_system,
                source_component=self.source_component
            )
            
            # Wait for the first heartbeat
            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            self.connected = True
            self.last_heartbeat = time.time()
            
            # Store basic vehicle info
            self.vehicle_system = self.master.mav.srcSystem
            self.vehicle_component = self.master.mav.srcComponent
            
            logger.info(f"Connected to system {self.vehicle_system} component {self.vehicle_component}")
            return True
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def update_telemetry(self) -> bool:
        """Update telemetry data from the vehicle"""
        if not self.connected:
            return False
            
        try:
            # Process all available messages
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                    
                self._process_message(msg)
                
            # Check for heartbeat timeout
            if time.time() - self.last_heartbeat > self.heartbeat_timeout:
                logger.warning("Heartbeat timeout")
                self.connected = False
                return False
                
            return True
            
        except Exception as e:
            logger.error(f"Error updating telemetry: {e}")
            self.connected = False
            return False
    
    def _process_message(self, msg):
        """Process incoming Mavlink messages"""
        msg_type = msg.get_type()
        
        if msg_type == 'HEARTBEAT':
            self.last_heartbeat = time.time()
            self.telemetry.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.telemetry.timestamp = time.time()
            
            # Convert mode from mavlink to string
            mode_mapping = mavutil.mode_mapping
            mode_num = msg.custom_mode
            mode_name = [k for k, v in mode_mapping.items() if v == mode_num]
            self.telemetry.mode = mode_name[0] if mode_name else 'UNKNOWN'
            
        elif msg_type == 'GLOBAL_POSITION_INT':
            self.telemetry.latitude = msg.lat / 1e7  # Convert from degE7 to degrees
            self.telemetry.longitude = msg.lon / 1e7  # Convert from degE7 to degrees
            self.telemetry.altitude = msg.alt / 1000.0  # Convert from mm to meters
            self.telemetry.heading = msg.hdg / 100.0  # Convert from centidegrees to degrees
            
        elif msg_type == 'VFR_HUD':
            self.telemetry.groundspeed = msg.groundspeed
            
        elif msg_type == 'SYS_STATUS':
            self.telemetry.battery_voltage = msg.voltage_battery / 1000.0  # mV to V
            self.telemetry.battery_remaining = msg.battery_remaining
    
    def arm(self) -> bool:
        """Arm the vehicle"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.vehicle_system, self.vehicle_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,   # Arm
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            return True
        except Exception as e:
            logger.error(f"Arming failed: {e}")
            return False
    
    def disarm(self) -> bool:
        """Disarm the vehicle"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.vehicle_system, self.vehicle_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                0,   # Disarm
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            return True
        except Exception as e:
            logger.error(f"Disarming failed: {e}")
            return False
    
    def set_mode(self, mode: str) -> bool:
        """Set flight mode"""
        if not self.connected:
            return False
            
        try:
            # Get mode mapping
            mode_mapping = mavutil.mode_mapping
            if mode not in mode_mapping:
                logger.error(f"Unknown mode: {mode}")
                return False
                
            mode_id = mode_mapping[mode]
            
            self.master.mav.set_mode_send(
                self.vehicle_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            return True
            
        except Exception as e:
            logger.error(f"Failed to set mode: {e}")
            return False
    
    def upload_mission(self, waypoints: List[Dict[str, Any]]) -> bool:
        """Upload a mission with waypoints"""
        if not self.connected:
            return False
            
        try:
            # Clear existing mission
            self.master.mav.mission_clear_all_send(
                self.vehicle_system, self.vehicle_component
            )
            
            # Wait for the clear to complete
            self.master.recv_match(type='MISSION_ACK', blocking=True)
            
            # Send mission count
            self.master.mav.mission_count_send(
                self.vehicle_system, self.vehicle_component,
                len(waypoints), 0  # 0 = mission, 1 = geofence, 2 = rally points
            )
            
            # Wait for the vehicle to request waypoints
            while True:
                msg = self.master.recv_match(type='MISSION_REQUEST', blocking=True)
                if msg.target_system == self.source_system and \
                   msg.target_component == self.source_component:
                    break
            
            # Send waypoints
            for i, wp in enumerate(waypoints):
                self.master.mav.mission_item_send(
                    self.vehicle_system, self.vehicle_component,
                    i,  # Sequence number
                    wp.get('frame', mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT),
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,  # Current
                    1 if i == 0 else 0,  # Autocontinue
                    wp.get('p1', 0), wp.get('p2', 0), wp.get('p3', 0),  # Parameters
                    wp.get('x', 0),  # Latitude
                    wp.get('y', 0),   # Longitude
                    wp.get('z', 10),  # Altitude (default 10m)
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                
                # Wait for the next request or completion
                msg = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'], blocking=True)
                if msg.get_type() == 'MISSION_ACK':
                    break
            
            logger.info("Mission uploaded successfully")
            return True
            
        except Exception as e:
            logger.error(f"Mission upload failed: {e}")
            return False
    
    def start_mission(self) -> bool:
        """Start the current mission"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.vehicle_system, self.vehicle_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0,  # Confirmation
                0, 0, 0, 0, 0, 0, 0  # Unused parameters
            )
            return True
        except Exception as e:
            logger.error(f"Failed to start mission: {e}")
            return False
    
    def get_parameters(self) -> Dict[str, Any]:
        """Request all parameters from the vehicle"""
        if not self.connected:
            return {}
            
        try:
            # Request parameter list
            self.master.param_fetch_all()
            
            # Wait for parameters
            while True:
                msg = self.master.recv_match(type=['PARAM_VALUE', 'PARAM_EXT_VALUE'], blocking=True)
                if msg.get_type() == 'PARAM_VALUE':
                    if msg.param_count > 0 and msg.param_index == msg.param_count - 1:
                        break
            
            # Get all parameters
            self.parameters = self.master.params
            return dict(self.parameters)
            
        except Exception as e:
            logger.error(f"Failed to get parameters: {e}")
            return {}
    
    def set_parameter(self, name: str, value: float) -> bool:
        """Set a parameter on the vehicle"""
        if not self.connected:
            return False
            
        try:
            self.master.param_set_send(
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                -1,  # All components
                -1   # All instances
            )
            return True
        except Exception as e:
            logger.error(f"Failed to set parameter {name}: {e}")
            return False
    
    def close(self):
        """Close the connection"""
        if self.master:
            self.master.close()
        self.connected = False
        
    def __del__(self):
        self.close()
