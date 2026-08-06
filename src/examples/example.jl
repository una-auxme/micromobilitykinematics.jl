using micromobilitykinematics
using GLMakie

## Steering geometry

max_varphi_config = (15.0, 1.0, 35.0)

steering = Steering(
    75.42675540883899,
    91.8286361513241,
    123.54626813005024,
    227.48110204522783,
)

## Suspension geometry

lower_wishbones = (
    LowerWishbone(
        id = :Left,
        bearing_rear = [0.0, 0.0, 0.0],
        bearing_distance_x = 74.0,
        bearing_front = [74.0, 0.0, 0.0],
        rotation_axis = [1.0, 0.0, 0.0],
        distance_to_joint_y = 140.0,
        distance_rotation_axis_to_lower_damper_fixture = 85.0,
        distance_to_joint_x = 37.0,
    ),
    LowerWishbone(
        id = :Right,
        bearing_rear = [0.0, 0.0, 0.0],
        bearing_distance_x = 74.0,
        bearing_front = [74.0, 0.0, 0.0],
        rotation_axis = [1.0, 0.0, 0.0],
        distance_to_joint_y = 140.0,
        distance_rotation_axis_to_lower_damper_fixture = 85.0,
        distance_to_joint_x = 37.0,
    ),
)

upper_wishbones = (
    UpperWishbone(
        id = :Left,
        bearing_rear = [0.0, 0.0, 139.0],
        bearing_distance_x = 74.0,
        bearing_front = [74.0, 0.0, 139.0],
        rotation_axis = [1.0, 0.0, 0.0],
        distance_to_joint_y = 140.0,
        distance_to_joint_x = 37.0,
        tiltx = 0.0,
        tilty = 0.0,
        tiltZ = 0.0,
    ),
    UpperWishbone(
        id = :Right,
        bearing_rear = [0.0, 0.0, 139.0],
        bearing_distance_x = 74.0,
        bearing_front = [74.0, 0.0, 139.0],
        rotation_axis = [1.0, 0.0, 0.0],
        distance_to_joint_y = 140.0,
        distance_to_joint_x = 37.0,
        tiltx = 0.0,
        tilty = 0.0,
        tiltZ = 0.0,
    ),
)

dampers = (
    Damper(
        id = :Left,
        nominal_length = 210.0,
        travel = 55.0,
        compression = 30.0,
        length_neutral_compression = 30.0,
        upper_fixture = [37.0, 30.0, 160.0],
    ),
    Damper(
        id = :Right,
        nominal_length = 210.0,
        travel = 55.0,
        compression = 30.0,
        length_neutral_compression = 30.0,
        upper_fixture = [37.0, 30.0, 160.0],
    ),
)

wheelmount = WheelMount(
    length = 139.0,
    camber_angle = 0.0,
    offset_x = 0.0,
    offset_y = 50.0,
    offset_z = 139.0 / 2,
    to_angle = 0.0,
)

suspension = Suspension(
    compressions = (30.0, 30.0),
    lowerwishbone = lower_wishbones,
    upperwishbone = upper_wishbones,
    damper = dampers,
    wheelmount = wheelmount,
)

chassis = Chassis()

## Evaluate kinematics

varphi_config = (0.0, 1.0, 0.0)

micromobilitykinematics.update!(varphi_config, steering, suspension)

steering.δi
steering.δo

## Optional optimization

lower_border = (50.0, 50.0, 70.0, 195.0)
upper_border = (100.0, 100.0, 200.0, 260.0)

optimization_domain = OptimizationDomain(
    max_varphi_config;
    varphi_x_range_deg = (0.0, 10.0),
    compression_range_percent = (10.0, 90.0),
    high_steering_compression_range_percent = (20.0, 70.0),
    high_steering_threshold_deg = 15.0,
)

start_parameters = random_search(
    upper_border,
    lower_border,
    max_varphi_config;
    suspension = suspension,
    chassis = chassis,
    domain = optimization_domain,
)
opt = optim_over_range(
    start_parameters...,
    max_varphi_config;
    suspension = suspension,
    chassis = chassis,
    domain = optimization_domain,
)
steering = opt.steering
micromobilitykinematics.update!(varphi_config, steering, suspension)

## Interactive GUI

gui_varphi_limits = (15.0, 5.0, 35.0)

launch_gui(gui_varphi_limits, chassis, steering, suspension; path = @__DIR__)
