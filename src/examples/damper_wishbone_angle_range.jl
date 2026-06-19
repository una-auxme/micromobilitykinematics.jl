using micromobilitykinematics
using DataFrames
using LinearAlgebra
using Printf

"""
    example_suspension()

Creates the same suspension geometry as `example.jl`.
If this script is included after a custom setup that defines `suspension`,
the custom suspension is used instead.
"""
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

unit(v) = v / norm(v)

function angle_between_deg(a, b)
    denominator = norm(a) * norm(b)
    denominator == 0 && return NaN
    return acosd(clamp(dot(a, b) / denominator, -1.0, 1.0))
end

function lower_wishbone_vector(suspension, side_index)
    lower = suspension.lowerwishbone[side_index]
    axis_point = lower.bearing_rear .+ unit(lower.rotation_axis) .* lower.distance_to_joint_x
    return lower.sphere_joint .- axis_point
end

function lower_damper_fixture_radius(suspension, side_index)
    lower = suspension.lowerwishbone[side_index]
    lower_fixture = suspension.damper[side_index].lower_fixture
    axis = unit(lower.rotation_axis)
    axis_point = lower.bearing_rear .+ dot(lower_fixture .- lower.bearing_rear, axis) .* axis
    return lower_fixture .- axis_point
end

function damper_wishbone_angles(suspension, side_index)
    damper = suspension.damper[side_index]
    damper_axis = damper.upper_fixture .- damper.lower_fixture

    return (
        damper_to_lower_wishbone_deg = angle_between_deg(
            damper_axis,
            lower_wishbone_vector(suspension, side_index),
        ),
        damper_to_lower_fixture_radius_deg = angle_between_deg(
            damper_axis,
            lower_damper_fixture_radius(suspension, side_index),
        ),
    )
end

"""
    damper_wishbone_angle_sweep(suspension; compression_step = 1.0)

Sweeps the full damper travel by setting compression from 0% to 100%.
The damper length is calculated by the package as:

    length = nominal_length - compression / 100 * travel

Returns a DataFrame with one row per side and compression step.
"""
function damper_wishbone_angle_sweep(suspension; compression_step = 1.0)
    suspension_sweep = deepcopy(suspension)
    compression_values = collect(0.0:compression_step:100.0)
    last(compression_values) == 100.0 || push!(compression_values, 100.0)
    rows = NamedTuple[]

    for compression_percent in compression_values
        for damper in suspension_sweep.damper
            damper.compression = compression_percent
        end

        suspensionkinematics!(suspension_sweep)

        for side_index in eachindex(suspension_sweep.damper)
            damper = suspension_sweep.damper[side_index]
            angles = damper_wishbone_angles(suspension_sweep, side_index)

            push!(rows, (
                side = damper.id,
                compression_percent = compression_percent,
                damper_length_mm = damper.length,
                damper_travel_used_mm = damper.nominal_length - damper.length,
                damper_to_lower_wishbone_deg = angles.damper_to_lower_wishbone_deg,
                damper_to_lower_fixture_radius_deg = angles.damper_to_lower_fixture_radius_deg,
            ))
        end
    end

    return DataFrame(rows)
end

function print_angle_summary(results)
    println("Damper/lower-wishbone relative angle over full damper travel")

    for side in unique(results.side)
        side_rows = results[results.side .== side, :]
        wishbone_min, wishbone_max = extrema(side_rows.damper_to_lower_wishbone_deg)
        fixture_min, fixture_max = extrema(side_rows.damper_to_lower_fixture_radius_deg)

        @printf(
            "%s: lower wishbone %.3f deg .. %.3f deg | lower fixture radius %.3f deg .. %.3f deg\n",
            string(side),
            wishbone_min,
            wishbone_max,
            fixture_min,
            fixture_max,
        )
    end
end

suspension_to_evaluate = isdefined(Main, :suspension) ? deepcopy(Main.suspension) : example_suspension()

results = damper_wishbone_angle_sweep(suspension_to_evaluate; compression_step = 1.0)
print_angle_summary(results)

println()
println("First rows:")
show(first(results, 8); allcols = true)
println()
