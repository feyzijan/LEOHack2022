from sat_controller import SatControllerInterface, sat_msgs
import math


class TeamController(SatControllerInterface):

    def team_init(self):
        self.logger.info("Initialisation started")

        self.challenge_no = 4 # Used to toggle manually resetting theta for challenge 2 after it exceeds n*2*pi

        # Integral Errors initialisation
        # The integral error is approximated by summing up all previous positional errors 
        self.e_x_integ = 0.0
        self.e_y_integ = 0.0
        self.e_o_integ = 0.0

        # Variables used to store the previous posititonal error for error derivative calculation
        self.e_x_prev = 0.0
        self.e_y_prev = 0.0
        self.e_o_prev = 0.0

        # Derivative Errors initialisation
        # Derivative Error is approximated by subtracting the previous error from the current
        self.e_x_dt = 0.0
        self.e_y_dt = 0.0
        self.e_o_dt = 0.0

        # Note: Derivative and Integral errors are approximates, and due to the small timesteps you may need lower Gain 
        # values for them not to be ridiculously high

        # Maximum velocities achieved in each direction - to be logged
        self.v_x_max_log = 0.0
        self.v_y_max_log = 0.0
        self.v_z_max_log = 0.0
        self.v_o_max_log = 0.0

        # Gain parameters for different challenges
        # These are tuned so that different types of approaches can be attempted: 
        # eg. overshoot in y direction then approach while in a orbital like motion

        # Positional Error Proportional Gain
        self.Kp = [[8.0,  6.0,  8.0 ], 
              [6.0,  10.5,  8.0],
              [0.8,  0.5,  5.0 ],
              [8.0,  6.0,  8.0 ] ]

        # Positional Error Derivative (approximation) Gain
        self.Kd = [[0.0, 0.0, 0.0 ],
              [0.0, 0.0, 0.0 ],
              [0.5, 0.0, 0.0 ],
              [0.0, 0.0, 0.0 ],]

        # Positional Error Integral (approximation) Gain
        self.Ki = [[0.01, 0.01,  0.01 ],
              [0.01, 0.03,  0.01 ],
              [0.003, 0.003, 0.2 ],
              [0.01, 0.01,  0.01 ] ]

        # Moving Satellite Velocity Gain - for damping
        self.Kdy = [[20.0,  20.0,  20.0],
              [20.0,  20.0,  20.0 ],
              [20.0,  20.0,  20.0 ],
              [20.0,  20.0,  20.0 ] ]
        
        # Overall gain 
        self.K = [[1.0,  1.0,  1.0],
             [1.0,  1.0,  1.0],
             [1.0,  1.0,  1.0],
             [1.0,  1.0,  1.0] ]

        self.counter = 0

        # Update team info
        team_info = sat_msgs.TeamInfo()
        team_info.teamName = "Team7"
        team_info.teamID = 7777

        # Return team info
        return team_info

    def team_run(self, system_state: sat_msgs.SystemState, satellite_state: sat_msgs.SatelliteState, dead_sat_state: sat_msgs.SatelliteState) -> sat_msgs.ControlMessage:
        """ Takes in a system state, satellite state """

        n = self.challenge_no - 1 

        # Log time and counter
        elapsed_time = system_state.elapsedTime.ToTimedelta()
        self.logger.info(f'Elapsed time: {elapsed_time}')
        self.counter += 1
        self.logger.info(f'Counter value: {self.counter}')
        

        # Create a thrust command message
        control_message = sat_msgs.ControlMessage()

        # Positional Errors and necessary offsets for docking  
        # Note: In the below lines the required x and y offsets are slightly modified based on the control parameters 
        # An ideal selection of control parameters should make the small adjustments below unnecessary, but life only long enough
        # to try so many control parameters
        # So x and y positional errors are slightly modifeid to compensate for non-optimal parameter choices
         
         
        # X offset to subtract from x error
        x_offset = -0.25 * math.sin(dead_sat_state.pose.theta) 
        
        # Challenge specific modifications
        if n == 1: # To make up for theta being weird in challenge 2
             x_offset *= -1
        elif n == 3:
            x_offset -= 0.03
        
        # Y offset to subtract from y error
        y_offset = 0.25 * math.cos(dead_sat_state.pose.theta) 

        # Challenge specific modifications
        if n == 1:
            y_offset += 0.03
        elif n == 2 or n == 3:
            y_offset += 0.05


        # Omega offset (so satellites face together)
        o_offset = math.pi

        # Below code is used to make up for theta not resetting to zero after teaching n*2*pi in Challenge 3
        if self.challenge_no == 2: 
            if dead_sat_state.pose.theta < -2*math.pi:
                dead_sat_state.pose.theta += 2* math.pi
            elif dead_sat_state.pose.theta > 2*math.pi:
                dead_sat_state.pose.theta -= 2* math.pi

        # Calculate positional errors
        e_x = dead_sat_state.pose.x - satellite_state.pose.x - x_offset
        e_y = dead_sat_state.pose.y - satellite_state.pose.y - y_offset 
        e_z = math.sqrt(e_x * e_x + (e_y + y_offset) * (e_y + y_offset)) 
        e_o = dead_sat_state.pose.theta - satellite_state.pose.theta -o_offset

        # Velocities
        v_x = satellite_state.twist.v_x 
        v_y = satellite_state.twist.v_y 
        v_o = satellite_state.twist.omega 
        v_z = math.sqrt(v_x * v_x + v_y * v_y)

        # Approximate Integral Errors
        self.e_x_integ += e_x 
        self.e_y_integ += e_y 
        self.e_o_integ += e_o 

        # Approximate Derivative Errors
        self.e_x_dt = e_x - self.e_x_prev 
        self.e_y_dt = e_y - self.e_y_prev 
        self.e_o_dt = e_o - self.e_o_prev 

        # Update variables that store previous positional errors
        self.e_x_prev = e_x
        self.e_y_prev = e_y
        self.e_o_prev = e_o
        
        ### PID Control Code
        control_message.thrust.f_x = self.K[n][0] * (self.Kp[n][0] * (e_x)  + self.Ki[n][0] * self.e_x_integ + self.Kd[n][0] * self.e_x_dt - self.Kdy[n][0] * v_x )
        control_message.thrust.f_y = self.K[n][1] * (self.Kp[n][1] * (e_y)  + self.Ki[n][1] * self.e_y_integ + self.Kd[n][1] * self.e_y_dt - self.Kdy[n][1] * v_y )
        control_message.thrust.tau = self.K[n][2] * (self.Kp[n][2] * (e_o)  + self.Ki[n][2] * self.e_o_integ + self.Kd[n][2] * self.e_o_dt - self.Kdy[n][2] * v_o )
        

        ########## Enforcing  velocity constraints ################

        # Set initial threshold values for thrust - to be updated
        thrust_x_max, thrust_y_max, thrust_tau_max = 10,10,10
        
        # Decide on state : 1- Far away, 2- Within 0.5m radius, 3- Docking
        if e_z > 0.5: 
            v_max_rel = 1
        elif e_z > 0.075:
            self.logger.info("--- Within sphere---")
            v_max_rel = 0.2
        else: 
            self.logger.info("---Just about to dock---")
            v_max_rel = 0.05
            #w_max_rel = 0.1 
        ##################
        
        # Get absolute velocities in x, y, z, o for both satellites
        v_x_sat, v_y_sat, v_o_sat = satellite_state.twist.v_x, satellite_state.twist.v_y, satellite_state.twist.omega
        v_x_dsat, v_y_dsat, v_o_sat = dead_sat_state.twist.v_x, dead_sat_state.twist.v_y, dead_sat_state.twist.omega
        v_z_sat_abs = math.sqrt(v_x_sat * v_x_sat + (v_y_sat) * (v_y_sat))
        v_z_dsat_abs = math.sqrt(v_x_dsat * v_x_dsat + (v_y_dsat) * (v_y_dsat))

        # Get relative velocity in z
        v_z_rel = abs(v_z_sat_abs - v_z_dsat_abs)

        if v_z_rel < v_max_rel: # Velocity constraint not met
            # Basically limit x and y velocities so their sum of squares cant go over the limit
            # Then apply the approximate maximum thrusts to limit the velocity 
            v_x_allowed = v_x_dsat + v_max_rel / 2
            v_y_allowed = v_y_dsat + v_max_rel / 2
    
            thrust_x_max = ( v_x_allowed - v_x_sat) * 20
            thrust_y_max = ( v_y_allowed - v_y_sat) * 20


        # Wait up a bit for simulation to load after starting
        if self.counter <15:
            control_message.thrust.f_x, control_message.thrust.f_y, control_message.thrust.tau = 0, 0, 0
        
        # Logs for debugging  
        self.logger.info(f" Offsets: X: {e_x + x_offset},  \n Y: {e_y + y_offset}, Z: {e_z} \n O: {e_o + o_offset} ")
        self.logger.info(f" Integral errors: X: {self.e_x_integ},  \n Y: {self.e_y_integ + 0.25} \n O: {self.e_o_integ} ")
        #self.logger.info(f" Velocity Offsets: X: {e_dx},  \n Y: {e_dy} \n O: {e_do} ")
        self.logger.info(f' Our satellite: {satellite_state}')
        self.logger.info(f' Dead satellite: {dead_sat_state}')
        self.logger.info(f' Our messsage {control_message}')
        

        # Log maximum velocities achieved for later inspection
        if abs(v_x) > abs(self.v_x_max_log):
            self.v_x_max_log = v_x
        if abs(v_y) > abs(self.v_y_max_log):
            self.v_y_max_log = v_y
        if abs(v_z) > abs(self.v_z_max_log):
            self.v_z_max_log = v_z
        
        self.logger.info(f" Max velocities: X: {self.v_x_max_log},  \n Y: {self.v_y_max_log}, Z: {self.v_z_max_log} ")


        # Return control message
        return control_message

    def team_reset(self) -> None:
        # Run any reset code
        pass


        # Max Velocity Enforcement in Function Form - Not used
    def check_max_vel(self,satellite_state: sat_msgs.SatelliteState, dead_sat_state: sat_msgs.SatelliteState):
        state = 0
        """ Decide where the satelite is
        1- Far away (v_max_rel = 2)
        2- Within the 0.5m radius (v_max_rel = 0.2)
        3- Within docking distance 0.25 square (v_max_rel = 0.05, w_max_rel = 0.1 """

        # Calculate z distance
        e_x = dead_sat_state.pose.x - satellite_state.pose.x 
        e_y = dead_sat_state.pose.y - satellite_state.pose.y -0.25
        e_o = dead_sat_state.pose.theta - satellite_state.pose.theta - math.pi
        e_z = math.sqrt(e_x * e_x + (e_y + 0.25) * (e_y + 0.25))
        #################3


        # Decide state
        if e_z > 0.5: 
            state = 1
            v_max_rel = 1
        elif e_z > 0.075: # Close
            self.logger.info(" --- Within sphere--- ")
            state = 2
            v_max_rel = 0.2
        else: # Real close
            self.logger.info(" ---Just about to dock--- ")
            state = 3
            v_max_rel = 0.05
            w_max_rel = 0.1 
        ##################
        
        # Get velocities
        v_x_sat, v_y_sat, v_o_sat = satellite_state.twist.v_x, satellite_state.twist.v_y, satellite_state.twist.omega
        v_x_dsat, v_y_dsat, v_o_sat = dead_sat_state.twist.v_x, dead_sat_state.twist.v_y, dead_sat_state.twist.omega

        # Get ratio between vx and vy - used to determine velocities if speed too fast
        v_ratio = v_x_sat / v_y_sat

        # Get relative z velocities
        v_z_sat_abs = math.sqrt(v_x_sat * v_x_sat + (v_y_sat) * (v_y_sat))
        v_z_dsat_abs = math.sqrt(v_x_dsat * v_x_dsat + (v_y_dsat) * (v_y_dsat))

        v_z_rel_current = abs(v_z_sat_abs - v_z_dsat_abs)

        # To do: predict beforehand whether v_z will reach threshold

        if v_z_rel < v_max_rel:
            pass
        else:
            v_y_allowed = math.sqrt( v_max_rel / (v_ratio * v_ratio + 1) ) 
            v_x_allowed = math.sqrt( v_max_rel - v_y_allowed * v_y_allowed)
            thrust_x_allowed = (v_x_sat- v_x_allowed ) * 20
            thrust_y_allowed = (v_y_sat- v_y_allowed ) * 20

        return thrust_x_allowed, thrust_y_allowed