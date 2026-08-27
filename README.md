Single Inverted Pendulum

Hardware assumptions:

These constants are specific to this rig, recalibrate them if you build your own:

CART_TRAVEL_METERS / CART_TRAVEL_COUNTS: physical rail length (0.40 m) and the raw encoder counts measured over that full travel (21028). Re-measure both for your rail and encoder.
PENDULUM_ZERO_RAW: the AS5600 raw reading with the pendulum hanging straight down. Read this value directly off your sensor at rest and swap it in.
K[4]: LQR gains [x, theta, x_dot, theta_dot], derived from this cart's mass, pendulum length/inertia, and motor force characteristics. These won't transfer to a different rig; re-derive from your own system model (or re-tune experimentally).
FORCE_TO_PWM / DEADBAND_PWM / MAX_PWM: map controller force output to motor PWM; depends on your motor/driver and supply voltage.
CART_SAFE_MARGIN: soft travel limit in encoder counts, set relative to wherever the cart is centered at power-on (no homing routine in this build).

Setup:
1. Center the cart by hand before powering on (position becomes the zero reference).
2. Hold the pendulum upright and still, it auto-arms within ARM_ANGLE/ARM_VEL of vertical.
3. Serial commands: k kill, space disarm, p print cart count, l start a 2000-sample angle log.
