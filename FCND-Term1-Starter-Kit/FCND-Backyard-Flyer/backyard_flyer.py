import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # waypoints counter, there will be 4 waypoints in order to make square shape
        self.waypoints_counter = 4

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # I implement waypoint_transition and disarming_transition calling under the local position callback
        # because I need the current refreshed local position information for these two transitions
        if self.flight_state == States.TAKEOFF:
            current_altitude = -1 * self.local_position[2]
            if (current_altitude >= 0.95 * self.target_position[2] and # be sure that we reached the target altitude which we set at takeoff transition
                np.linalg.norm(self.local_velocity[2]) < 0.1):         # also be sure that the drone is not moving on z-axis, this check prevents immediate action after takeoff  
                self.all_waypoints = self.calculate_box()              # calculate the waypoints
                self.waypoint_transition()                             # we can transient to waypoint
        
        elif self.flight_state == States.WAYPOINT:
            if self.waypoints_counter != 0:
                self.waypoint_transition()
                if np.linalg.norm(self.local_position[0:2] - self.target_position[0:2]) < 0.5 : # control whether we reach the target position or not
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.1:                          # control the two horizontal velocity in order to make sharper transition to next square corner
                        print("we reach", 4 - self.waypoints_counter, "target")
                        self.waypoints_counter -= 1                                             # if we reach current target position then take next target position if available
                

        elif self.flight_state == States.LANDING:
            if (-1 * self.local_position[2]) < 0.1:   # if we are in LANDING state and altitude is less than 0.1 meter than we can disarm
                self.disarming_transition()



    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        if self.flight_state == States.WAYPOINT:                   # if we are in WAYPOINT state and there is no waypoint left to visit, then we can land 
            if self.waypoints_counter == 0:                        # we land home position because we set this point as square's last corner
                if np.linalg.norm(self.local_velocity[0:2]) < 0.1: # check the horizontal velocities in order to land safely
                    self.landing_transition()


    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.flight_state == States.MANUAL and not self.armed:  # if state is manual and not armed then transient to arming
            self.arming_transition()
        elif self.flight_state == States.ARMING and self.armed:    # if state is arming and we are armed then transient to takeoff

            self.takeoff_transition()
        elif self.flight_state == States.DISARMING and not self.armed: # if state is disarming and we are disarmed then transient to manual mode
            self.manual_transition()

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        # set waypoints array and keep altitude info from takeoff state
        waypoints = ([5.0, 0, self.target_position[2]],
                     [5.0, 5.0, self.target_position[2]],
                     [0, 5.0, self.target_position[2]],
                     [0, 0, self.target_position[2]]) # square's last point is home location where we takeoff
        return waypoints

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        # I had to add this guard in order to take action on simulation coordinates
        # otherwise the drone would have tried to go actual zero position and out of control
        if (self.global_position[0] != 0.0 and
            self.global_position[1] != 0.0 and
            self.global_position[2] != 0.0):
                self.set_home_position(self.global_position[0], 
                                       self.global_position[1],
                                       self.global_position[2])
                self.take_control()
                self.arm()
                self.flight_state = States.ARMING
                print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        self.target_position[2] = 3.0 
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.target_position = self.all_waypoints[4 - self.waypoints_counter]
        self.cmd_position(self.target_position[0],
                          self.target_position[1],
                          self.target_position[2],
                          0)

        self.flight_state = States.WAYPOINT
        #print("waypoint transition")        

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        self.disarm()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
