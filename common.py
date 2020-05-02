import numpy as np
import math

HARDWARE_ALARM =    0x0001
OS_ALARM =          0x0002
PRESSURE_ALARM =    0x0004
FLOW_ALARM =        0x0008
VOL_ALARM =         0x0010
PPEAK_ALARM =       0x0020
PEEP_ALARM =        0x0040

# Return an angle between -math.pi and math.pi.
# double theta : (IN) Value.
# Return : The converted angle.
def fmod_2PI(theta):
	return (((theta % (2 * math.PI)) + 3 * math.PI) % (2 * math.PI)) - math.PI

# Return an angle between -180 and 180.
# double theta : (IN) Value.
# Return : The converted angle.
def fmod_360(theta):
	return (((theta % (2 * 180.0)) + 3 * 180.0) % (2 * 180.0)) - 180.0

# Remember to reset piz to 0 whenever this control is re-enabled.
# direction_coef : depending on the type of robot, we need to invert depending on the direction of the movement, 
# set to -1 if needed or 1 otherwise.
def PID_control(z_bar, z_bar_prev, z, dz, piz, direction_coef, dt,
	Kp, Kd, Ki, up_max, ud_max, ui_max,
	u_min, u_max, error_min, error_max, dz_max):
	u = 0
	error = z_bar-z
	derivative = -dz
	integral = piz
	if (z_bar != z_bar_prev): integral = 0
	if (error > error_max):
		u = np.sign(direction_coef)*u_max;
		integral = 0;
	elif (error < error_min):
		u = np.sign(direction_coef)*u_min;
		integral = 0;
	else:
		if (abs(Kp*error) > up_max): u += np.sign(direction_coef)*sign(Kp*error)*up_max;
		else: u += sign(direction_coef)*Kp*error;
		if (abs(Kd*derivative/dz_max) > ud_max): u += np.sign(direction_coef)*np.sign(Kd*derivative/dz_max)*ud_max;
		else: u += sign(direction_coef)*Kd*derivative/dz_max; # /dz_max to try to normalize...
		if (abs(Ki*integral) > ui_max): u += np.sign(direction_coef)*np.sign(Ki*integral)*ui_max;
		else: u += np.sign(direction_coef)*Ki*integral;
		integral = integral+error*dt
	if (u < u_min): u = u_min
	elif (u > u_max): u = u_max
	piz = integral;
	return u, piz
