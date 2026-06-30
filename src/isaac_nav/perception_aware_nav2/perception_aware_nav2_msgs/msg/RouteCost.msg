std_msgs/Header header
string robot_id
string trial_id
float64 j_total
float64 time_term
float64 obs_term
float64 comms_term
float64 safety_term
float64 energy_term
float64 lambda_obs
float64 lambda_comms
float64 lambda_safety
float64 lambda_energy
float64 path_length_m
float64 nominal_speed_mps
float64 soc_fraction

# --- term-driving inputs (self-sufficient logging / training features) ---
int32 n_obstacles           # active tracked polygons this cycle (drives obs_term)
float64 nearest_obstacle_m  # closest approach of the plan to any active track; -1 = none
float64 rsrp_dbm            # latest comms RSRP behind comms_term; NaN = no link_stats yet
float64 jitter_ms          # latest comms jitter behind comms_term; NaN = no link_stats yet
float64 min_ttc_s          # min predicted time-to-collision w/ dynamic obstacles; -1 = no threat