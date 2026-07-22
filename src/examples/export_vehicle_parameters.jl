using micromobilitykinematics
using GLMakie
using LinearAlgebra
using Printf

const GUIExt = Base.get_extension(micromobilitykinematics, :micromobilitykinematicsGUIExt)
const OUT_DIR = joinpath(@__DIR__, "vehicle_parameter_export")
mkpath(OUT_DIR)

const COMPONENT_HEADER = [
    "order",
    "group",
    "component",
    "side",
    "parameter",
    "value",
    "unit",
    "source_or_note",
]

const VEHICLE_HEADER = [
    "order",
    "group",
    "parameter",
    "value",
    "unit",
    "source_or_note",
]

const CALCULATED_HEADER = [
    "order",
    "group",
    "quantity",
    "side",
    "reference_or_sweep",
    "value",
    "min",
    "max",
    "delta_from_neutral",
    "unit",
    "source_or_note",
]

function example_steering()
    return Steering(
        62.81680256916951,
        100.00000099935133,
        108.80559236847354,
        227.8382026583041,
    )
end

function example_suspension()
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

    return Suspension(
        compressions = (30.0, 30.0),
        lowerwishbone = lower_wishbones,
        upperwishbone = upper_wishbones,
        damper = dampers,
        wheelmount = wheelmount,
    )
end

unit_vector(v) = v ./ norm(v)

function angle_between_deg(a, b)
    denominator = norm(a) * norm(b)
    denominator <= eps(Float64) && return NaN
    return acosd(clamp(dot(a, b) / denominator, -1.0, 1.0))
end

function format_number(value)
    value isa Integer && return string(value)
    value isa Real || return string(value)
    !isfinite(Float64(value)) && return string(value)
    return @sprintf("%.6f", Float64(value))
end

function format_value(value)
    value === nothing && return "not set"
    value isa Symbol && return string(value)
    value isa AbstractArray && return "[" * join(format_value.(collect(value)), ", ") * "]"
    value isa Tuple && return "(" * join(format_value.(collect(value)), ", ") * ")"
    value isa Real && return format_number(value)
    return string(value)
end

function finite_values(values)
    flat = Float64[]
    for value in values
        if value isa AbstractArray || value isa Tuple
            append!(flat, finite_values(value))
        elseif value isa Real && isfinite(Float64(value))
            push!(flat, Float64(value))
        end
    end
    return flat
end

function range_values(values)
    finite = finite_values(values)
    isempty(finite) && return (NaN, NaN)
    return extrema(finite)
end

function delta_range(values, neutral)
    finite = finite_values(values)
    isempty(finite) && return "n/a"
    return @sprintf("%+.6f / %+.6f", minimum(finite) - neutral, maximum(finite) - neutral)
end

function csv_escape(value)
    text = format_value(value)
    text = replace(text, "\"" => "\"\"")
    return "\"" * text * "\""
end

function write_csv(path, header, rows)
    open(path, "w") do io
        println(io, join(csv_escape.(header), ";"))
        for row in rows
            println(io, join(csv_escape.(row), ";"))
        end
    end
end

function add_component!(rows, group, component, side, parameter, value, unit = "", note = "")
    push!(rows, Any[length(rows) + 1, group, component, side, parameter, value, unit, note])
end

function add_vehicle!(rows, group, parameter, value, unit = "", note = "")
    push!(rows, Any[length(rows) + 1, group, parameter, value, unit, note])
end

function add_calc!(rows, group, quantity, side, reference, value, min_value = "", max_value = "", delta = "", unit = "", note = "")
    push!(rows, Any[length(rows) + 1, group, quantity, side, reference, value, min_value, max_value, delta, unit, note])
end

function add_calc_range!(rows, group, quantity, side, reference, values, unit = ""; neutral = nothing, note = "")
    min_value, max_value = range_values(values)
    delta = neutral === nothing ? "" : delta_range(values, Float64(neutral))
    add_calc!(rows, group, quantity, side, reference, "", min_value, max_value, delta, unit, note)
end

function gui_function(name)
    return getfield(GUIExt, Symbol(name))
end

function delta_value(steering, side_index)
    symbol = side_index == 1 ? Symbol(Char(0x03b4), "o") : Symbol(Char(0x03b4), "i")
    return getfield(steering, symbol)
end

function lower_wishbone_vector(suspension, side_index)
    lower = suspension.lowerwishbone[side_index]
    axis_point = lower.bearing_rear .+ unit_vector(lower.rotation_axis) .* lower.distance_to_joint_x
    return lower.sphere_joint .- axis_point
end

function lower_damper_fixture_radius(suspension, side_index)
    lower = suspension.lowerwishbone[side_index]
    lower_fixture = suspension.damper[side_index].lower_fixture
    axis = unit_vector(lower.rotation_axis)
    axis_point = lower.bearing_rear .+ dot(lower_fixture .- lower.bearing_rear, axis) .* axis
    return lower_fixture .- axis_point
end

function wheelcarrier_vector(suspension, side_index)
    return suspension.upperwishbone[side_index].sphere_joint .- suspension.lowerwishbone[side_index].sphere_joint
end

function damper_wishbone_angles(suspension, side_index)
    damper = suspension.damper[side_index]
    damper_axis = damper.upper_fixture .- damper.lower_fixture
    return (
        damper_to_lower_wishbone = angle_between_deg(damper_axis, lower_wishbone_vector(suspension, side_index)),
        damper_to_lower_fixture_radius = angle_between_deg(damper_axis, lower_damper_fixture_radius(suspension, side_index)),
    )
end

function lower_wishbone_wheelcarrier_angle(suspension, side_index)
    return angle_between_deg(lower_wishbone_vector(suspension, side_index), wheelcarrier_vector(suspension, side_index))
end

function compression_values(; step = 1.0)
    values = collect(0.0:step:100.0)
    last(values) == 100.0 || push!(values, 100.0)
    return values
end

function angle_sweeps_over_compression(suspension; step = 1.0)
    result = Dict{Tuple{Symbol,String},Vector{Float64}}()
    for side in (:Left, :Right)
        result[(side, "damper_to_lower_wishbone")] = Float64[]
        result[(side, "damper_to_lower_fixture_radius")] = Float64[]
        result[(side, "lower_wishbone_to_wheelcarrier")] = Float64[]
    end

    for compression in compression_values(; step = step)
        suspension_work = deepcopy(suspension)
        for damper in suspension_work.damper
            damper.compression = compression
        end
        micromobilitykinematics.suspensionkinematics!(suspension_work)
        for side_index in 1:2
            side = side_index == 1 ? :Left : :Right
            angles = damper_wishbone_angles(suspension_work, side_index)
            push!(result[(side, "damper_to_lower_wishbone")], angles.damper_to_lower_wishbone)
            push!(result[(side, "damper_to_lower_fixture_radius")], angles.damper_to_lower_fixture_radius)
            push!(result[(side, "lower_wishbone_to_wheelcarrier")], lower_wishbone_wheelcarrier_angle(suspension_work, side_index))
        end
    end
    return result
end

function compression_values_for_varphi_z(varphi_z, default_values, high_values; threshold = 15.0)
    abs(varphi_z) > threshold && return high_values
    return default_values
end

function sweep_range(min_value, max_value, step)
    values = collect(float(min_value):float(step):float(max_value))
    last(values) == float(max_value) || push!(values, float(max_value))
    return values
end

function track_lever_tie_rod_angle(steering, side_index)
    mount = Float64.(steering.track_lever_mounting_points_ucs[side_index])
    joint = Float64.(steering.circle_joints[side_index])
    sphere = Float64.(steering.sphere_joints[side_index])
    return angle_between_deg(mount .- joint, sphere .- joint)
end

function rotational_component_tie_rod_angle(steering, side_index)
    mount = Float64.(steering.vec_z_rotational)
    joint = Float64.(steering.sphere_joints[side_index])
    outer_joint = Float64.(steering.circle_joints[side_index])
    return angle_between_deg(mount .- joint, outer_joint .- joint)
end

function tie_rod_angle_sweep(steering, suspension, angle_function;
                             side_index = 1,
                             varphi_x_range = (0.0, 10.0),
                             varphi_y = 1.0,
                             varphi_z_range = (-35.0, 35.0),
                             compression_range = (10.0, 90.0),
                             high_compression_range = (20.0, 70.0),
                             high_varphi_z_threshold = 15.0,
                             angle_step = 1.0,
                             compression_step = 1.0)
    varphi_x_values = sweep_range(varphi_x_range[1], varphi_x_range[2], angle_step)
    varphi_z_values = sweep_range(varphi_z_range[1], varphi_z_range[2], angle_step)
    default_compressions = sweep_range(compression_range[1], compression_range[2], compression_step)
    high_compressions = sweep_range(high_compression_range[1], high_compression_range[2], compression_step)

    angles = Float64[]
    failures = 0
    total = 0

    for varphi_x in varphi_x_values, varphi_z in varphi_z_values
        compr_values = compression_values_for_varphi_z(
            varphi_z,
            default_compressions,
            high_compressions;
            threshold = high_varphi_z_threshold,
        )
        for compression in compr_values
            total += 1
            steering_work = deepcopy(steering)
            suspension_work = deepcopy(suspension)
            suspension_work.damper[1].compression = compression
            suspension_work.damper[2].compression = compression
            try
                micromobilitykinematics.update!((varphi_x, varphi_y, varphi_z), steering_work, suspension_work)
                push!(angles, angle_function(steering_work, side_index))
            catch
                failures += 1
            end
        end
    end

    return (angles = angles, failures = failures, total = total)
end

function steering_surface_sweep(steering, suspension, max_varphi_config; step = 1.0)
    varphi_x_max, varphi_y, varphi_z_max = max_varphi_config
    delta_i = Float64[]
    delta_o = Float64[]
    failures = 0

    for varphi_x in 0.0:step:varphi_x_max, varphi_z in 0.0:step:varphi_z_max
        steering_work = deepcopy(steering)
        suspension_work = deepcopy(suspension)
        try
            micromobilitykinematics.update!((varphi_x, varphi_y, varphi_z), steering_work, suspension_work)
            push!(delta_i, getfield(steering_work, Symbol(Char(0x03b4), "i")))
            push!(delta_o, getfield(steering_work, Symbol(Char(0x03b4), "o")))
        catch
            failures += 1
        end
    end

    return (delta_i = delta_i, delta_o = delta_o, failures = failures)
end

function add_component_rows!(rows, steering, suspension, chassis)
    rc = steering.rotational_component
    add_component!(rows, "01 Steering", "RotationalComponent", "both", "x_rotational_radius", rc.x_rotational_radius, "mm")
    add_component!(rows, "01 Steering", "RotationalComponent", "both", "z_rotational_radius", rc.z_rotational_radius, "mm")
    add_component!(rows, "01 Steering", "RotationalComponent", "both", "to_joint_pivot_point", rc.to_joint_pivot_point, "mm", "constructive tie-rod pivot offset per side")
    add_component!(rows, "01 Steering", "RotationalComponent", "both", "distance_between_joint_pivot_points", rc.distance_between_joint_pivot_points, "mm")
    add_component!(rows, "01 Steering", "TrackLever", "both", "length", steering.track_lever.length, "mm")
    add_component!(rows, "01 Steering", "TieRod", "both", "length", steering.tie_rod.length, "mm")

    for side_index in 1:2
        side = side_index == 1 ? "Left" : "Right"
        add_component!(rows, "01 Steering", "Steering UCS", side, "wishbone_ucs_position", steering.wishbone_ucs_position[side_index], "mm", "position of wishbone UCS in steering UCS")
    end

    for side_index in 1:2
        side = side_index == 1 ? "Left" : "Right"
        lower = suspension.lowerwishbone[side_index]
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "bearing_rear", lower.bearing_rear, "mm")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "bearing_distance_x", lower.bearing_distance_x, "mm")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "bearing_front", lower.bearing_front, "mm")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "rotation_axis", lower.rotation_axis, "-")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "distance_to_joint_y", lower.distance_to_joint_y, "mm")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "distance_rotation_axis_to_lower_damper_fixture", lower.distance_rotation_axis_to_lower_damper_fixture, "mm")
        add_component!(rows, "02 Suspension", "LowerWishbone", side, "distance_to_joint_x", lower.distance_to_joint_x, "mm")

        upper = suspension.upperwishbone[side_index]
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "bearing_rear", upper.bearing_rear, "mm")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "bearing_distance_x", upper.bearing_distance_x, "mm")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "bearing_front", upper.bearing_front, "mm")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "rotation_axis", upper.rotation_axis, "-")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "distance_to_joint_y", upper.distance_to_joint_y, "mm")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "distance_to_joint_x", upper.distance_to_joint_x, "mm")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "tiltx", upper.tiltx, "deg")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "tilty", upper.tilty, "deg")
        add_component!(rows, "02 Suspension", "UpperWishbone", side, "tiltZ", upper.tiltZ, "deg")

        damper = suspension.damper[side_index]
        neutral_compression = (damper.nominal_length - damper.length_neutral) / damper.travel * 100
        add_component!(rows, "02 Suspension", "Damper", side, "nominal_length", damper.nominal_length, "mm")
        add_component!(rows, "02 Suspension", "Damper", side, "travel", damper.travel, "mm")
        add_component!(rows, "02 Suspension", "Damper", side, "compression", damper.compression, "%")
        add_component!(rows, "02 Suspension", "Damper", side, "length_neutral_compression", neutral_compression, "%")
        add_component!(rows, "02 Suspension", "Damper", side, "upper_fixture", damper.upper_fixture, "mm")
    end

    wheelmount = suspension.wheelmount
    add_component!(rows, "02 Suspension", "WheelMount", "both", "length", wheelmount.length, "mm")
    add_component!(rows, "02 Suspension", "WheelMount", "both", "camber_angle", wheelmount.camper_angle, "deg")
    add_component!(rows, "02 Suspension", "WheelMount", "both", "offset_x", wheelmount.offset_x, "mm")
    add_component!(rows, "02 Suspension", "WheelMount", "both", "offset_y", wheelmount.offset_y, "mm")
    add_component!(rows, "02 Suspension", "WheelMount", "both", "offset_z", wheelmount.offset_z, "mm")
    add_component!(rows, "02 Suspension", "WheelMount", "both", "to_angle", wheelmount.to_angle, "deg")

    add_component!(rows, "03 Chassis", "Chassis", "vehicle", "width", chassis.width, "mm", "distance between lower wishbone rear bearings")
    add_component!(rows, "03 Chassis", "Chassis", "vehicle", "length", isdefined(chassis, :length) ? chassis.length : "not set", "mm")
    add_component!(rows, "03 Chassis", "Chassis", "vehicle", "radius", isdefined(chassis, :radius) ? chassis.radius : "not set", "mm")
end

function add_vehicle_rows!(rows, chassis, steering, suspension, max_varphi_config, gui_varphi_limits, varphi_config)
    measurement = Measurements(chassis, steering)
    add_vehicle!(rows, "01 Vehicle dimensions", "front track width from Measurements", measurement.track_width, "mm", "chassis.width + 2 * abs(left wheel UCS y)")
    add_vehicle!(rows, "01 Vehicle dimensions", "wheelbase", measurement.wheel_base, "mm", "hard-coded in Measurements")
    add_vehicle!(rows, "01 Vehicle dimensions", "desired turning radius", measurement.turning_radius, "mm", "hard-coded in Measurements")
    add_vehicle!(rows, "01 Vehicle dimensions", "lower wishbone bearing width", chassis.width, "mm")

    add_vehicle!(rows, "02 Operating point", "neutral/current varphi_config", varphi_config, "deg", "(varphi_x, varphi_y, varphi_z)")
    add_vehicle!(rows, "02 Operating point", "neutral compression left/right", (suspension.damper[1].compression, suspension.damper[2].compression), "%")
    add_vehicle!(rows, "03 Sweep setup", "max_varphi_config", max_varphi_config, "deg", "(varphi_x_max, fixed varphi_y, varphi_z_max)")
    add_vehicle!(rows, "03 Sweep setup", "gui_varphi_limits", gui_varphi_limits, "deg", "(varphi_x_max, varphi_y_max, varphi_z_max)")
    add_vehicle!(rows, "03 Sweep setup", "symmetric compression sweep", "0 .. 100", "%", "used for suspension ranges")
    add_vehicle!(rows, "03 Sweep setup", "realistic tie-rod sweep compression", "10 .. 90; 20 .. 70 for abs(varphi_z) > 15 deg", "%", "varphi_x 0..10 deg, varphi_z -35..35 deg")
end

function add_current_position_rows!(rows, steering, suspension)
    for side_index in 1:2
        side = side_index == 1 ? "Left" : "Right"
        add_calc!(rows, "01 Neutral/current positions", "wheel_ucs_position", side, "current 30% compression", steering.wheel_ucs_position[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "track_lever_mounting_point_ucs", side, "current 30% compression", steering.track_lever_mounting_points_ucs[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "sphere_joint tie rod on rotational component", side, "current varphi_config", steering.sphere_joints[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "sphere_joint tie rod neutral", side, "varphi_x = 0, varphi_z = 0", steering.sphere_joints_neutral[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "circle_joint track lever", side, "current varphi_config", steering.circle_joints[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "circle_joint track lever neutral", side, "varphi_x = 0, varphi_z = 0", steering.circle_joints_neutral[side_index], "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "lower wishbone sphere joint", side, "current 30% compression", suspension.lowerwishbone[side_index].sphere_joint, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "lower wishbone sphere joint neutral", side, "30% neutral compression", suspension.lowerwishbone[side_index].sphere_joint_neutral, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "upper wishbone sphere joint", side, "current 30% compression", suspension.upperwishbone[side_index].sphere_joint, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "upper wishbone sphere joint neutral", side, "30% neutral compression", suspension.upperwishbone[side_index].sphere_joint_neutral, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "damper lower fixture", side, "current 30% compression", suspension.damper[side_index].lower_fixture, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "damper lower fixture neutral", side, "30% neutral compression", suspension.damper[side_index].lower_fixture_neutral, "", "", "", "mm")

        wheel_center = gui_function("wheel_center_vehicle_position")(steering, suspension, side_index)
        steered_wheel_center = gui_function("steered_wheel_center_vehicle_position")(steering, suspension, side_index)
        wheel_axis = gui_function("wheel_axis_vehicle_direction")(suspension, side_index)
        camber = gui_function("wheel_camber_angle")(suspension, side_index)
        add_calc!(rows, "01 Neutral/current positions", "wheel center", side, "current 30% compression", wheel_center, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "steered wheel center", side, "current varphi_config", steered_wheel_center, "", "", "", "mm")
        add_calc!(rows, "01 Neutral/current positions", "wheel axis direction", side, "current 30% compression", wheel_axis, "", "", "", "-")
        add_calc!(rows, "02 Instantaneous values", "camber", side, "current 30% compression", camber, "", "", "", "deg")
        add_calc!(rows, "02 Instantaneous values", "wheel angle delta", side, "current varphi_config", delta_value(steering, side_index), "", "", "", "deg", "Left uses delta_o, Right uses delta_i in this model")

        track_lever_vector = Float64.(steering.circle_joints_neutral[side_index]) .- Float64.(steering.track_lever_mounting_points_ucs[side_index])
        add_calc!(rows, "02 Instantaneous values", "track lever vector", side, "neutral", track_lever_vector, "", "", "", "mm")
        add_calc!(rows, "02 Instantaneous values", "track lever angle to wheel axis", side, "neutral", angle_between_deg(track_lever_vector, wheel_axis), "", "", "", "deg")
        add_calc!(rows, "02 Instantaneous values", "tie rod / track lever angle", side, "current varphi_config", track_lever_tie_rod_angle(steering, side_index), "", "", "", "deg")
        add_calc!(rows, "02 Instantaneous values", "tie rod / rotational component lever angle", side, "current varphi_config", rotational_component_tie_rod_angle(steering, side_index), "", "", "", "deg")
    end

    add_calc!(rows, "01 Neutral/current positions", "vec_x_rotational", "both", "current varphi_config", steering.vec_x_rotational, "", "", "", "mm")
    add_calc!(rows, "01 Neutral/current positions", "vec_z_rotational", "both", "current varphi_config", steering.vec_z_rotational, "", "", "", "mm")
    add_calc!(rows, "01 Neutral/current positions", "vec_x_rotational_neutral", "both", "varphi_x = 0, varphi_z = 0", steering.vec_x_rotational_neutral, "", "", "", "mm")
    add_calc!(rows, "01 Neutral/current positions", "vec_z_rotational_neutral", "both", "varphi_x = 0, varphi_z = 0", steering.vec_z_rotational_neutral, "", "", "", "mm")
end

function add_sweep_rows!(rows, chassis, steering, suspension, max_varphi_config, varphi_config)
    measurement = Measurements(chassis, steering)
    add_calc!(rows, "02 Instantaneous values", "track width", "vehicle", "current 30% compression", measurement.track_width, "", "", "", "mm")

    turning_radius = gui_function("turning_radius")(chassis, steering)
    add_calc!(rows, "02 Instantaneous values", "outer wheel track radius", "vehicle", "current varphi_config", turning_radius, "", "", "", "mm")

    try
        deviation = micromobilitykinematics.ackermann_deviation(varphi_config, chassis, deepcopy(steering), deepcopy(suspension))
        ratio_unsigned = gui_function("ackermannratio")(varphi_config, chassis, deepcopy(steering), deepcopy(suspension); signed = false)
        ratio_signed = gui_function("ackermannratio")(varphi_config, chassis, deepcopy(steering), deepcopy(suspension); signed = true)
        add_calc!(rows, "02 Instantaneous values", "Ackermann deviation", "vehicle", "current varphi_config", deviation, "", "", "", "mm")
        add_calc!(rows, "02 Instantaneous values", "Ackermann ratio unsigned", "vehicle", "current varphi_config", ratio_unsigned, "", "", "", "%")
        add_calc!(rows, "02 Instantaneous values", "Ackermann ratio signed", "vehicle", "current varphi_config", ratio_signed, "", "", "", "%")
    catch err
        add_calc!(rows, "02 Instantaneous values", "Ackermann at current varphi_config", "vehicle", "current varphi_config", "not defined", "", "", "", "", sprint(showerror, err))
    end

    compression_vals, left_path, right_path = gui_function("wheel_center_path")(steering, suspension; step_size = 1.0)
    neutral_index = findfirst(==(30.0), compression_vals)

    for (side, path) in (("Left", left_path), ("Right", right_path))
        xs = [Float64(point[1]) for point in path]
        ys = [Float64(point[2]) for point in path]
        zs = [Float64(point[3]) for point in path]
        add_calc_range!(rows, "03 Symmetric compression sweep", "wheel center x", side, "compression 0..100%", xs, "mm"; neutral = xs[neutral_index])
        add_calc_range!(rows, "03 Symmetric compression sweep", "wheel center y", side, "compression 0..100%", ys, "mm"; neutral = ys[neutral_index])
        add_calc_range!(rows, "03 Symmetric compression sweep", "wheel center z", side, "compression 0..100%", zs, "mm"; neutral = zs[neutral_index])
    end

    compression_vals, track_width = gui_function("track_width_over_compression")(steering, suspension; step_size = 1.0)
    neutral_track = track_width[findfirst(==(30.0), compression_vals)]
    add_calc_range!(rows, "03 Symmetric compression sweep", "track width", "vehicle", "compression 0..100%", track_width, "mm"; neutral = neutral_track, note = "delta_from_neutral is track-width change")

    compression_vals, motion_ratio = gui_function("damper_motion_ratio")(steering, suspension; step_size = 1.0)
    add_calc_range!(rows, "03 Symmetric compression sweep", "damper motion ratio", "Left", "compression 0..100%", motion_ratio, "mm/mm", note = "damper travel per vertical wheel-center travel")

    angle_ranges = angle_sweeps_over_compression(suspension; step = 1.0)
    for side in (:Left, :Right)
        side_text = string(side)
        add_calc_range!(rows, "03 Symmetric compression sweep", "damper to lower wishbone angle", side_text, "compression 0..100%", angle_ranges[(side, "damper_to_lower_wishbone")], "deg")
        add_calc_range!(rows, "03 Symmetric compression sweep", "damper to lower fixture radius angle", side_text, "compression 0..100%", angle_ranges[(side, "damper_to_lower_fixture_radius")], "deg")
        add_calc_range!(rows, "03 Symmetric compression sweep", "lower wishbone to wheel carrier angle", side_text, "compression 0..100%", angle_ranges[(side, "lower_wishbone_to_wheelcarrier")], "deg")
    end

    surface = steering_surface_sweep(steering, suspension, max_varphi_config; step = 1.0)
    add_calc_range!(rows, "04 Steering input sweep", "delta_i surface", "Right/inner", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", surface.delta_i, "deg"; note = "$(surface.failures) failed steering states")
    add_calc_range!(rows, "04 Steering input sweep", "delta_o surface", "Left/outer", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", surface.delta_o, "deg"; note = "$(surface.failures) failed steering states")

    radii = gui_function("steering_radii")(chassis, deepcopy(steering), deepcopy(suspension), max_varphi_config; step_size = 1.0)
    add_calc_range!(rows, "04 Steering input sweep", "outer wheel track radius surface", "vehicle", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", radii, "mm")

    ack_unsigned = gui_function("ackermannratio_surface")(chassis, deepcopy(steering), deepcopy(suspension), max_varphi_config; signed = false, step_size = 1.0)
    ack_signed = gui_function("ackermannratio_surface")(chassis, deepcopy(steering), deepcopy(suspension), max_varphi_config; signed = true, step_size = 1.0)
    ack_dev = gui_function("ackermann_deviation_surface")(chassis, deepcopy(steering), deepcopy(suspension), max_varphi_config; step_size = 1.0)
    add_calc_range!(rows, "04 Steering input sweep", "Ackermann ratio unsigned surface", "vehicle", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", ack_unsigned, "%")
    add_calc_range!(rows, "04 Steering input sweep", "Ackermann ratio signed surface", "vehicle", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", ack_signed, "%")
    add_calc_range!(rows, "04 Steering input sweep", "Ackermann deviation surface", "vehicle", "varphi_x 0..15, varphi_z 0..35, varphi_y 1", ack_dev, "mm")

    delta_left = gui_function("left_wheel_delta_vs_compression_" * string(Char(0x03d5)) * "z")(
        0.0,
        max_varphi_config[2],
        max_varphi_config[3],
        deepcopy(steering),
        deepcopy(suspension);
        fixed_right_compression = suspension.damper[2].compression,
        step_size = 1.0,
    )
    add_calc_range!(rows, "05 Combined compression/steering sweep", "left wheel angle change", "Left", "left compression 0..100%, varphi_z 0..35, right compression fixed 30%", delta_left, "deg")

    compression_vals, varphi_z_vals, left_x, left_y, left_z, right_x, right_y, right_z =
        gui_function("wheel_center_surface")(0.0, max_varphi_config[2], max_varphi_config[3], deepcopy(steering), deepcopy(suspension); compression_step = 5.0)
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface x", "Left", "compression 0..100%, varphi_z -35..35", left_x, "mm")
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface y", "Left", "compression 0..100%, varphi_z -35..35", left_y, "mm")
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface z", "Left", "compression 0..100%, varphi_z -35..35", left_z, "mm")
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface x", "Right", "compression 0..100%, varphi_z -35..35", right_x, "mm")
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface y", "Right", "compression 0..100%, varphi_z -35..35", right_y, "mm")
    add_calc_range!(rows, "05 Combined compression/steering sweep", "wheel center surface z", "Right", "compression 0..100%, varphi_z -35..35", right_z, "mm")

    roll_varphi = (0.0, max_varphi_config[2], 15.0)
    roll_values, left_camber, right_camber, left_wheel_angle, right_wheel_angle, roll_track_width, roll_ackermann =
        gui_function("roll_kinematics")(roll_varphi, chassis, deepcopy(steering), deepcopy(suspension); signed = true, step_size = 1.0)
    add_calc_range!(rows, "06 Roll sweep", "camber", "Left", "left compression 0..100%, right compression 100..0%, varphi_z 15", left_camber, "deg")
    add_calc_range!(rows, "06 Roll sweep", "camber", "Right", "left compression 0..100%, right compression 100..0%, varphi_z 15", right_camber, "deg")
    add_calc_range!(rows, "06 Roll sweep", "wheel angle delta", "Left", "left compression 0..100%, right compression 100..0%, varphi_z 15", left_wheel_angle, "deg")
    add_calc_range!(rows, "06 Roll sweep", "wheel angle delta", "Right", "left compression 0..100%, right compression 100..0%, varphi_z 15", right_wheel_angle, "deg")
    add_calc_range!(rows, "06 Roll sweep", "track width", "vehicle", "left compression 0..100%, right compression 100..0%, varphi_z 15", roll_track_width, "mm")
    add_calc_range!(rows, "06 Roll sweep", "Ackermann ratio signed", "vehicle", "left compression 0..100%, right compression 100..0%, varphi_z 15", roll_ackermann, "%")

    track_angle_sweep = tie_rod_angle_sweep(steering, suspension, track_lever_tie_rod_angle; side_index = 1)
    rotational_angle_sweep = tie_rod_angle_sweep(steering, suspension, rotational_component_tie_rod_angle; side_index = 1)
    add_calc_range!(rows, "07 Tie rod construction sweeps", "tie rod / track lever angle", "Left", "varphi_x 0..10, varphi_z -35..35, compression 10..90 or 20..70", track_angle_sweep.angles, "deg"; note = "$(track_angle_sweep.failures) failed of $(track_angle_sweep.total) states")
    add_calc_range!(rows, "07 Tie rod construction sweeps", "tie rod / rotational component lever angle", "Left", "varphi_x 0..10, varphi_z -35..35, compression 10..90 or 20..70", rotational_angle_sweep.angles, "deg"; note = "$(rotational_angle_sweep.failures) failed of $(rotational_angle_sweep.total) states")
end

function main()
    max_varphi_config = (15.0, 1.0, 35.0)
    gui_varphi_limits = (15.0, 5.0, 35.0)
    varphi_config = (0.0, 1.0, 0.0)

    steering = example_steering()
    suspension = example_suspension()
    chassis = Chassis()

    micromobilitykinematics.update!(varphi_config, steering, suspension)

    component_rows = Any[]
    vehicle_rows = Any[]
    calculated_rows = Any[]

    add_component_rows!(component_rows, steering, suspension, chassis)
    add_vehicle_rows!(vehicle_rows, chassis, steering, suspension, max_varphi_config, gui_varphi_limits, varphi_config)
    add_current_position_rows!(calculated_rows, steering, suspension)
    add_sweep_rows!(calculated_rows, chassis, steering, suspension, max_varphi_config, varphi_config)

    write_csv(joinpath(OUT_DIR, "01_bauteile.csv"), COMPONENT_HEADER, component_rows)
    write_csv(joinpath(OUT_DIR, "02_fahrzeug.csv"), VEHICLE_HEADER, vehicle_rows)
    write_csv(joinpath(OUT_DIR, "03_berechnet.csv"), CALCULATED_HEADER, calculated_rows)

    println("Wrote CSV export to: ", OUT_DIR)
end

main()
