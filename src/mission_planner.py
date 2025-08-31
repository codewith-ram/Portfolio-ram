"""
Mission planning and management for ArduPilot GCS
"""
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Any, Union
from dataclasses import dataclass, asdict
import time

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class Waypoint:
    """Represents a mission waypoint"""
    latitude: float
    longitude: float
    altitude: float  # meters AMSL
    speed: float = 0.0  # m/s, 0 = use default
    hold_time: int = 0  # seconds
    accept_radius: float = 5.0  # meters
    pass_radius: float = 2.0  # meters
    yaw_angle: float = 0.0  # degrees
    waypoint_type: str = 'WAYPOINT'  # WAYPOINT, TAKEOFF, LAND, etc.
    autocontinue: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert waypoint to dictionary"""
        return {
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'speed': self.speed,
            'hold_time': self.hold_time,
            'accept_radius': self.accept_radius,
            'pass_radius': self.pass_radius,
            'yaw_angle': self.yaw_angle,
            'waypoint_type': self.waypoint_type,
            'autocontinue': self.autocontinue
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Waypoint':
        """Create waypoint from dictionary"""
        return cls(**data)

class MissionPlanner:
    """Handles mission planning and management"""
    
    def __init__(self, mavlink_handler: 'MavlinkHandler'):
        """
        Initialize the mission planner
        
        Args:
            mavlink_handler: Instance of MavlinkHandler for communication
        """
        self.mav = mavlink_handler
        self.waypoints: List[Waypoint] = []
        self.current_mission = ""
        self.mission_dir = Path("missions")
        self.mission_dir.mkdir(exist_ok=True)
    
    def create_new_mission(self, name: str = None) -> str:
        """
        Create a new mission
        
        Args:
            name: Optional mission name
            
        Returns:
            str: Mission ID
        """
        self.waypoints = []
        if name:
            self.current_mission = name
        else:
            self.current_mission = f"mission_{int(time.time())}"
        return self.current_mission
    
    def add_waypoint(self, waypoint: Waypoint) -> int:
        """
        Add a waypoint to the current mission
        
        Args:
            waypoint: Waypoint to add
            
        Returns:
            int: Index of the added waypoint
        """
        self.waypoints.append(waypoint)
        return len(self.waypoints) - 1
    
    def insert_waypoint(self, index: int, waypoint: Waypoint) -> bool:
        """
        Insert a waypoint at the specified index
        
        Args:
            index: Index to insert at
            waypoint: Waypoint to insert
            
        Returns:
            bool: True if successful, False otherwise
        """
        if 0 <= index <= len(self.waypoints):
            self.waypoints.insert(index, waypoint)
            return True
        return False
    
    def remove_waypoint(self, index: int) -> bool:
        """
        Remove a waypoint at the specified index
        
        Args:
            index: Index of waypoint to remove
            
        Returns:
            bool: True if successful, False otherwise
        """
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)
            return True
        return False
    
    def clear_mission(self) -> None:
        """Clear all waypoints from the current mission"""
        self.waypoints = []
    
    def save_mission(self, filename: str = None) -> str:
        """
        Save the current mission to a file
        
        Args:
            filename: Optional filename (without .json extension)
            
        Returns:
            str: Path to the saved mission file
        """
        if not filename:
            filename = f"{self.current_mission}.json"
        elif not filename.endswith('.json'):
            filename += '.json'
            
        filepath = self.mission_dir / filename
        
        mission_data = {
            'name': self.current_mission,
            'created': time.strftime('%Y-%m-%d %H:%M:%S'),
            'waypoints': [wp.to_dict() for wp in self.waypoints]
        }
        
        with open(filepath, 'w') as f:
            json.dump(mission_data, f, indent=2)
            
        logger.info(f"Mission saved to {filepath}")
        return str(filepath)
    
    def load_mission(self, filename: str) -> bool:
        """
        Load a mission from a file
        
        Args:
            filename: Name of the mission file to load
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not filename.endswith('.json'):
            filename += '.json'
            
        filepath = self.mission_dir / filename
        
        try:
            with open(filepath, 'r') as f:
                mission_data = json.load(f)
                
            self.current_mission = mission_data.get('name', '')
            self.waypoints = [
                Waypoint.from_dict(wp) for wp in mission_data.get('waypoints', [])
            ]
            
            logger.info(f"Loaded mission '{self.current_mission}' with {len(self.waypoints)} waypoints")
            return True
            
        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            return False
    
    def list_missions(self) -> List[str]:
        """
        List all available missions
        
        Returns:
            List[str]: List of mission filenames
        """
        return [f.stem for f in self.mission_dir.glob('*.json')]
    
    def upload_mission_to_vehicle(self) -> bool:
        """
        Upload the current mission to the vehicle
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.waypoints:
            logger.warning("No waypoints in mission")
            return False
            
        # Convert waypoints to MAVLink format
        mav_waypoints = []
        
        for i, wp in enumerate(self.waypoints):
            mav_wp = {
                'x': wp.latitude,
                'y': wp.longitude,
                'z': wp.altitude,
                'p1': wp.speed,  # Speed to use after this waypoint
                'p2': wp.accept_radius,
                'p3': wp.pass_radius,
                'p4': wp.yaw_angle,
                'frame': 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                'autocontinue': 1 if wp.autocontinue else 0
            }
            
            # Set command based on waypoint type
            if wp.waypoint_type == 'TAKEOFF':
                mav_wp['command'] = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif wp.waypoint_type == 'LAND':
                mav_wp['command'] = mavutil.mavlink.MAV_CMD_NAV_LAND
            elif wp.waypoint_type == 'RETURN_TO_LAUNCH':
                mav_wp['command'] = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            elif wp.waypoint_type == 'LOITER_TIME':
                mav_wp['command'] = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
                mav_wp['p1'] = wp.hold_time  # Hold time in seconds
            else:  # Default to regular waypoint
                mav_wp['command'] = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            
            mav_waypoints.append(mav_wp)
        
        # Upload mission
        return self.mav.upload_mission(mav_waypoints)
    
    def download_mission_from_vehicle(self) -> bool:
        """
        Download mission from the vehicle
        
        Returns:
            bool: True if successful, False otherwise
        """
        # This would be implemented to download waypoints from the vehicle
        # For now, we'll just return False as this is a placeholder
        logger.warning("Mission download not yet implemented")
        return False
    
    def start_mission(self) -> bool:
        """
        Start the current mission on the vehicle
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.mav.start_mission()
    
    def get_mission_progress(self) -> Dict[str, Any]:
        """
        Get the current mission progress
        
        Returns:
            Dict with mission progress information
        """
        # This would be implemented to get the current mission progress
        # For now, return a placeholder
        return {
            'current_waypoint': 0,
            'total_waypoints': len(self.waypoints),
            'distance_to_next': 0.0,
            'time_to_next': 0.0
        }
