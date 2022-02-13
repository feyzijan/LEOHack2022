from sat_controller import SatControllerInterface, sat_msgs

import math

# Team code is written as an implementation of various methods
# within the the generic SatControllerInterface class.
# If you would like to see how this class works, look in sat_control/sat_controller

# Specifically, init, run, and reset

class TeamController(SatControllerInterface):
    """ Team control code """

    def team_init(self):
        """ Runs any team based initialization """
        self.logger.info("Initialisation started")
        # Run any initialization you need

        self.e_x_integ = 0.0
        self.e_y_integ = 0.0
        self.e_o_integ = 0.0

        self.e_x_prev = 0.0
        self.e_y_prev = 0.0
        self.e_o_prev = 0.0

        self.e_x_dt = 0.0
        self.e_y_dt = 0.0
        self.e_o_dt = 0.0

        self.v_x_max_log = 0
        self.v_y_max_log = 0
        self.v_z_max_log = 0

        # Example of persistant data
        self.counter = 0

        # Example of logging
        self.logger.info("Initialized :)")
        self.logger.warning("Warning...")
        self.logger.error("Error!")

        # Update team info
        team_info = sat_msgs.TeamInfo()
        team_info.teamName = "Team7"
        team_info.teamID = 1111

        # Return team info
        return team_info

    def team_run(self, system_state: sat_msgs.SystemState, satellite_state: sat_msgs.SatelliteState, dead_sat_state: sat_msgs.SatelliteState) -> sat_msgs.ControlMessage:
        """ Takes in a system state, satellite state """

        print(dead_sat_state)

        # Get timedelta from elapsed time
        elapsed_time = system_state.elapsedTime.ToTimedelta()
        self.logger.info(f'Elapsed time: {elapsed_time}')

        # Example of persistant data
        self.counter += 1

        # Example of logging
        self.logger.info(f'Counter value: {self.counter}')
        

        # Create a thrust command message
        control_message = sat_msgs.ControlMessage()

        ##### Get errors

        # Positional Error !!! Paste in relevant challenge's code
        x_offset = -0.25 * math.sin(dead_sat_state.pose.theta) - 0.03
        y_offset = 0.25 * math.cos(dead_sat_state.pose.theta) + 0.05
        o_offset = math.pi
        e_x = dead_sat_state.pose.x - satellite_state.pose.x - x_offset
        e_y = dead_sat_state.pose.y - satellite_state.pose.y - y_offset # 0.25
        e_o = dead_sat_state.pose.theta - satellite_state.pose.theta -o_offset
        e_z = math.sqrt(e_x * e_x + (e_y + y_offset) * (e_y + y_offset))
        #################################################
        
        if self.challenge_no == 2: # To make up for inverse angles
            if dead_sat_state.pose.theta < -2*math.pi:
                dead_sat_state.pose.theta += 2* math.pi
            elif dead_sat_state.pose.theta > 2*math.pi:
                dead_sat_state.pose.theta -= 2* math.pi

        e_o = dead_sat_state.pose.theta - satellite_state.pose.theta -o_offset
        e_z = math.sqrt(e_x * e_x + (e_y + y_offset) * (e_y + y_offset))

        # Velocities
        v_x = satellite_state.twist.v_x 
        v_y = satellite_state.twist.v_y 
        v_o = satellite_state.twist.omega 
        v_z = math.sqrt(v_x * v_x + v_y * v_y)



        ####### PD controller ##########
        Kp = [8.0,  6.0,  8.0]
        Kd = [0.1,0.1,0.1]
        Ki = [0.01, 0.01,  0.01]
        Kd = [0.0,  0.0,  0.0]
        Kdy = [20.0,  20.0,  20.0]

        K = [1.0,  1.0,  1.0]


        self.e_x_integ += e_x 
        self.e_y_integ += e_y 
        self.e_o_integ += e_o 

        self.e_x_dt = e_x - self.e_x_prev 
        self.e_y_dt = e_y - self.e_y_prev 
        self.e_o_dt = e_o - self.e_o_prev 

        # Update previous error values
        self.e_x_prev = e_x
        self.e_y_prev = e_y
        self.e_o_prev = e_o


        control_message.thrust.f_x = K[0] * (Kp[0] * (e_x)  + Ki[0] * self.e_x_integ + Kd[0] * self.e_x_dt - Kdy[0] * v_x )
        control_message.thrust.f_y = K[1] * (Kp[1] * (e_y)  + Ki[1] * self.e_y_integ + Kd[1] * self.e_y_dt - Kdy[1] * v_y )
        control_message.thrust.tau = K[2] * (Kp[2] * (e_o)  + Ki[2] * self.e_o_integ + Kd[2] * self.e_o_dt - Kdy[2] * v_o )
        
        #################################


        ########## Enforcing  velocity constraints ################

        # Initial high values for thrust to be updated
        thrust_x_max, thrust_y_max, thrust_tau_max = 10,10,10
        
        # Decide on state : 1- Far away, 2- Within 0.5m radius, 3- Docking
        if e_z > 0.5: 
            v_max_rel = 1
        elif e_z > 0.075: # Close
            self.logger.info("--- Within sphere---")
            v_max_rel = 0.2
        else: # Real close
            self.logger.info("---Just about to dock---")
            v_max_rel = 0.05
            #w_max_rel = 0.1 
        ##################
        
        # Get absolute velocities in x, y, z
        v_x_sat, v_y_sat, v_o_sat = satellite_state.twist.v_x, satellite_state.twist.v_y, satellite_state.twist.omega
        v_x_dsat, v_y_dsat, v_o_sat = dead_sat_state.twist.v_x, dead_sat_state.twist.v_y, dead_sat_state.twist.omega
        v_z_sat_abs = math.sqrt(v_x_sat * v_x_sat + (v_y_sat) * (v_y_sat))
        v_z_dsat_abs = math.sqrt(v_x_dsat * v_x_dsat + (v_y_dsat) * (v_y_dsat))

        # Get relative velocity in z
        v_z_rel = abs(v_z_sat_abs - v_z_dsat_abs)

        if v_z_rel < v_max_rel:
            v_x_allowed = v_x_dsat + v_max_rel / 2
            v_y_allowed = v_y_dsat + v_max_rel / 2
    
            thrust_x_max = ( v_x_allowed - v_x_sat) * 20
            thrust_y_max = (v_y_allowed - v_y_sat) * 20


        # Wait up for simulation to load after starting
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


    # In function form - not used
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


    def team_reset(self) -> None:
        # Run any reset code
        pass