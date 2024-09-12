from math import cos, pi, atan, tan, copysign, pi

DEG_2_RAD = pi/180.0
FIXED_AXIS_ANGLE_SHIFT = 0.349


def evaluate_driving_kinematics(frontShaftRate, rearShaftRate, frontAxisTurn, rearAxisTurn):
    r_platform = 0.2976  # wheels radius platform (const)
    lf = 0.47  # distance to front axis (const)
    lr = 0.47  # distance to rear axis (const)

    # Wheels rotation rates
    dfi1 = frontShaftRate/2 # Front left wheel
    dfi2 = frontShaftRate/2 # Front right wheel
    dfi3 = rearShaftRate/2 # Rear left wheel
    dfi4 = rearShaftRate/2 # Rear right wheel
    tauf = -(frontAxisTurn*DEG_2_RAD - FIXED_AXIS_ANGLE_SHIFT)  # median angle of the front axis rotation
    taur = -(rearAxisTurn*DEG_2_RAD - FIXED_AXIS_ANGLE_SHIFT) # median angle of the rear axis rotation

    slip = atan( (lf * tan(taur) - lr * tan(tauf)) / (lf + lr) )

    velocity = 1 / 4 * (dfi1 + dfi2 + dfi3 + dfi4) * r_platform * cos(slip)
    rate = velocity / (lf + lr) * (tan(tauf) - tan(taur))

    return velocity, rate