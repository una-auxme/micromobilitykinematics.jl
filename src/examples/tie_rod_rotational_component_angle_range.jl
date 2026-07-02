using micromobilitykinematics
using GLMakie
using LinearAlgebra
using Printf

"""
    tie_rod_rotational_component_example_steering()

Creates the same steering geometry as `example.jl`.
"""
function tie_rod_rotational_component_example_steering()
    return Steering(
        57.4050864963812,
        100.0000009999905,
        109.196240211308,
        229.7228503290388,
    )
end

"""
    tie_rod_rotational_component_example_suspension()

Creates the same suspension geometry as `example.jl`.
"""
function tie_rod_rotational_component_example_suspension()
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

tie_rod_rotational_component_unit(v) = v ./ norm(v)

function tie_rod_rotational_component_angle_between_deg(a, b)
    denominator = norm(a) * norm(b)
    denominator <= eps(Float64) && return NaN
    return acosd(clamp(dot(a, b) / denominator, -1.0, 1.0))
end

function sweep_values(min_value, max_value, step_size)
    step_size <= 0 && error("step size must be greater than zero")
    values = collect(float(min_value):float(step_size):float(max_value))
    last(values) == float(max_value) || push!(values, float(max_value))
    return values
end

function compression_values_for_phi_z(phi_z,
                                        default_compression_values,
                                        high_steering_compression_values;
                                        high_steering_phi_z_threshold_deg = 15.0)
    if abs(phi_z) > high_steering_phi_z_threshold_deg
        return high_steering_compression_values
    end

    return default_compression_values
end

function local_reference_lever_basis(reference_lever_mount_to_joint; vehicle_z = [0.0, 0.0, 1.0])
    x_axis = tie_rod_rotational_component_unit(reference_lever_mount_to_joint)
    z_candidate = vehicle_z .- dot(vehicle_z, x_axis) .* x_axis

    if norm(z_candidate) <= eps(Float64)
        z_candidate = [0.0, 1.0, 0.0] .- dot([0.0, 1.0, 0.0], x_axis) .* x_axis
    end

    z_axis = tie_rod_rotational_component_unit(z_candidate)
    y_axis = tie_rod_rotational_component_unit(cross(z_axis, x_axis))

    return (x = x_axis, y = y_axis, z = z_axis)
end

function to_local(vector, basis)
    return [
        dot(vector, basis.x),
        dot(vector, basis.y),
        dot(vector, basis.z),
    ]
end

function tie_rod_rotational_component_geometry(steering, side_index)
    mount = Float64.(steering.vec_z_rotational)
    joint = Float64.(steering.sphere_joints[side_index])
    outer_joint = Float64.(steering.circle_joints[side_index])

    reference_lever_mount_to_joint = joint .- mount
    reference_lever_joint_to_mount = mount .- joint
    tie_rod_joint_to_outer_joint = outer_joint .- joint

    basis = local_reference_lever_basis(reference_lever_mount_to_joint)

    return (
        reference_lever_length_mm = norm(reference_lever_mount_to_joint),
        angle_deg = tie_rod_rotational_component_angle_between_deg(reference_lever_joint_to_mount, tie_rod_joint_to_outer_joint),
        tie_rod_local = to_local(tie_rod_joint_to_outer_joint, basis),
    )
end

function pose_row(phi_x, phi_y, phi_z, suspension, side)
    return (
        side = side,
        phi_x_deg = phi_x,
        phi_y_deg = phi_y,
        phi_z_deg = phi_z,
        compression_percent = suspension.damper[1].compression,
        left_damper_length_mm = suspension.damper[1].length,
        right_damper_length_mm = suspension.damper[2].length,
    )
end

function failed_pose_row(phi_x, phi_y, phi_z, compression, side)
    return (
        side = side,
        phi_x_deg = phi_x,
        phi_y_deg = phi_y,
        phi_z_deg = phi_z,
        compression_percent = compression,
    )
end

mutable struct TieRodRotationalComponentSummary
    side::Symbol
    side_index::Int
    min_angle_deg::Float64
    max_angle_deg::Float64
    min_angle_pose::Any
    max_angle_pose::Any
    reference_direction::Vector{Float64}
    max_reference_direction_deviation_deg::Float64
    max_reference_direction_deviation_pose::Any
    optimal_direction::Vector{Float64}
    optimal_required_angle_deg::Float64
    optimal_required_pose::Any
    optimal_reference_offset_deg::Float64
    optimal_lateral_min_deg::Float64
    optimal_lateral_max_deg::Float64
    optimal_lateral_min_pose::Any
    optimal_lateral_max_pose::Any
    optimal_vertical_min_deg::Float64
    optimal_vertical_max_deg::Float64
    optimal_vertical_min_pose::Any
    optimal_vertical_max_pose::Any
    direction_sum::Vector{Float64}
    directions::Vector{Vector{Float64}}
    direction_poses::Vector{Any}
    evaluated_count::Int
    failed_count::Int
    failed_poses::Vector{Any}
    failed_reason_counts::Dict{String,Int}
    plotted_count::Int
    reference_lever_length_mm::Float64
end

function TieRodRotationalComponentSummary(side, side_index, reference_direction, reference_lever_length_mm)
    return TieRodRotationalComponentSummary(
        side,
        side_index,
        Inf,
        -Inf,
        nothing,
        nothing,
        reference_direction,
        -Inf,
        nothing,
        copy(reference_direction),
        NaN,
        nothing,
        NaN,
        Inf,
        -Inf,
        nothing,
        nothing,
        Inf,
        -Inf,
        nothing,
        nothing,
        zeros(3),
        Vector{Float64}[],
        Any[],
        0,
        0,
        Any[],
        Dict{String,Int}(),
        0,
        reference_lever_length_mm,
    )
end

function update_summary!(summary, geometry, pose)
    direction = tie_rod_rotational_component_unit(geometry.tie_rod_local)
    reference_deviation = tie_rod_rotational_component_angle_between_deg(direction, summary.reference_direction)

    summary.direction_sum .+= direction
    push!(summary.directions, direction)
    push!(summary.direction_poses, pose)
    summary.evaluated_count += 1

    if geometry.angle_deg < summary.min_angle_deg
        summary.min_angle_deg = geometry.angle_deg
        summary.min_angle_pose = pose
    end

    if geometry.angle_deg > summary.max_angle_deg
        summary.max_angle_deg = geometry.angle_deg
        summary.max_angle_pose = pose
    end

    if reference_deviation > summary.max_reference_direction_deviation_deg
        summary.max_reference_direction_deviation_deg = reference_deviation
        summary.max_reference_direction_deviation_pose = pose
    end

    return direction
end

function err_info_label(prefix, err_info)
    err_info === nothing && return nothing
    err_info.type === nothing && return nothing

    return "$(prefix) $(err_info.id): $(err_info.type) $(err_info.msg)"
end

function kinematics_failure_reason(err, steering, suspension)
    steering_reason = err_info_label("steering", steering.err_info)
    steering_reason !== nothing && return steering_reason

    suspension_reason = err_info_label("suspension", suspension.err_info)
    suspension_reason !== nothing && return suspension_reason

    return "$(typeof(err)): $(sprint(showerror, err))"
end

function record_failure!(summary, pose, reason)
    summary.failed_count += 1
    push!(summary.failed_poses, pose)
    summary.failed_reason_counts[reason] = get(summary.failed_reason_counts, reason, 0) + 1

    return nothing
end

function max_direction_deviation(axis, directions)
    min_dot = 1.0

    for direction in directions
        min_dot = min(min_dot, dot(axis, direction))
    end

    return acosd(clamp(min_dot, -1.0, 1.0))
end

function farthest_direction(axis, directions)
    worst_direction = directions[1]
    min_dot = dot(axis, worst_direction)

    for direction in directions
        direction_dot = dot(axis, direction)

        if direction_dot < min_dot
            min_dot = direction_dot
            worst_direction = direction
        end
    end

    return worst_direction
end

function perpendicular_basis(axis)
    helper = abs(dot(axis, [0.0, 0.0, 1.0])) < 0.9 ? [0.0, 0.0, 1.0] : [0.0, 1.0, 0.0]
    first_axis = tie_rod_rotational_component_unit(cross(axis, helper))
    second_axis = tie_rod_rotational_component_unit(cross(axis, first_axis))

    return first_axis, second_axis
end

function local_refine_neutral_direction(initial_axis, directions; step_deg = 20.0, min_step_deg = 0.02)
    best_axis = tie_rod_rotational_component_unit(initial_axis)
    best_angle = max_direction_deviation(best_axis, directions)
    step = step_deg

    while step >= min_step_deg
        improved = false
        first_axis, second_axis = perpendicular_basis(best_axis)

        for azimuth in range(0.0, 360.0; length = 25)[1:end-1]
            tangent_direction = cosd(azimuth) .* first_axis .+ sind(azimuth) .* second_axis
            candidate_axis = tie_rod_rotational_component_unit(best_axis .* cosd(step) .+ tangent_direction .* sind(step))
            candidate_angle = max_direction_deviation(candidate_axis, directions)

            if candidate_angle + 1e-9 < best_angle
                best_axis = candidate_axis
                best_angle = candidate_angle
                improved = true
            end
        end

        improved || (step *= 0.5)
    end

    return best_axis, best_angle
end

function optimal_neutral_direction(directions, reference_direction, direction_sum)
    isempty(directions) && return copy(reference_direction), NaN

    candidates = Vector{Float64}[]
    norm(direction_sum) > eps(Float64) && push!(candidates, tie_rod_rotational_component_unit(direction_sum))
    push!(candidates, copy(reference_direction))

    first_worst = farthest_direction(reference_direction, directions)
    second_worst = farthest_direction(first_worst, directions)
    pair_sum = first_worst .+ second_worst
    norm(pair_sum) > eps(Float64) && push!(candidates, tie_rod_rotational_component_unit(pair_sum))

    best_axis = candidates[1]
    best_angle = max_direction_deviation(best_axis, directions)

    for candidate in Iterators.drop(candidates, 1)
        candidate_angle = max_direction_deviation(candidate, directions)

        if candidate_angle < best_angle
            best_axis = candidate
            best_angle = candidate_angle
        end
    end

    return local_refine_neutral_direction(best_axis, directions)
end

function optimal_axis_component_basis(optimal_axis; local_z = [0.0, 0.0, 1.0])
    axis = tie_rod_rotational_component_unit(optimal_axis)
    vertical_candidate = local_z .- dot(local_z, axis) .* axis

    if norm(vertical_candidate) <= eps(Float64)
        vertical_candidate = [0.0, 1.0, 0.0] .- dot([0.0, 1.0, 0.0], axis) .* axis
    end

    vertical = tie_rod_rotational_component_unit(vertical_candidate)
    lateral = tie_rod_rotational_component_unit(cross(vertical, axis))

    return (axis = axis, lateral = lateral, vertical = vertical)
end

function direction_component_angles_deg(direction, basis)
    axial_component = dot(direction, basis.axis)
    lateral_component = dot(direction, basis.lateral)
    vertical_component = dot(direction, basis.vertical)

    return (
        lateral = rad2deg(atan(lateral_component, axial_component)),
        vertical = rad2deg(atan(vertical_component, axial_component)),
    )
end

function finalize_summary!(summary)
    optimal_direction, required_angle = optimal_neutral_direction(
        summary.directions,
        summary.reference_direction,
        summary.direction_sum,
    )

    summary.optimal_direction = optimal_direction
    summary.optimal_required_angle_deg = required_angle
    summary.optimal_reference_offset_deg = tie_rod_rotational_component_angle_between_deg(
        optimal_direction,
        summary.reference_direction,
    )

    worst_angle = -Inf
    worst_pose = nothing
    component_basis = optimal_axis_component_basis(optimal_direction)

    for (direction, pose) in zip(summary.directions, summary.direction_poses)
        angle = tie_rod_rotational_component_angle_between_deg(optimal_direction, direction)
        component_angles = direction_component_angles_deg(direction, component_basis)

        if angle > worst_angle
            worst_angle = angle
            worst_pose = pose
        end

        if component_angles.lateral < summary.optimal_lateral_min_deg
            summary.optimal_lateral_min_deg = component_angles.lateral
            summary.optimal_lateral_min_pose = pose
        end

        if component_angles.lateral > summary.optimal_lateral_max_deg
            summary.optimal_lateral_max_deg = component_angles.lateral
            summary.optimal_lateral_max_pose = pose
        end

        if component_angles.vertical < summary.optimal_vertical_min_deg
            summary.optimal_vertical_min_deg = component_angles.vertical
            summary.optimal_vertical_min_pose = pose
        end

        if component_angles.vertical > summary.optimal_vertical_max_deg
            summary.optimal_vertical_max_deg = component_angles.vertical
            summary.optimal_vertical_max_pose = pose
        end
    end

    summary.optimal_required_angle_deg = worst_angle
    summary.optimal_required_pose = worst_pose

    return summary
end

function side_index_from_symbol(side)
    side == :Left && return 1
    side == :Right && return 2
    error("side must be :Left or :Right")
end

function reference_geometry(phi_y, steering, suspension, side_index; reference_compression_percent = 30.0)
    steering_reference = deepcopy(steering)
    suspension_reference = deepcopy(suspension)

    suspension_reference.damper[1].compression = reference_compression_percent
    suspension_reference.damper[2].compression = reference_compression_percent
    micromobilitykinematics.update!((0.0, phi_y, 0.0), steering_reference, suspension_reference)

    return tie_rod_rotational_component_geometry(steering_reference, side_index)
end

function add_plot_vector!(
    plot_segments,
    plot_endpoints,
    plot_angles,
    summary,
    geometry;
    display_tie_rod_length_mm = 70.0,
    normalize_tie_rod_vectors_for_plot = true,
)
    tie_rod_local = geometry.tie_rod_local
    tie_rod_plot_vector = normalize_tie_rod_vectors_for_plot ?
                          tie_rod_rotational_component_unit(tie_rod_local) .* display_tie_rod_length_mm :
                          tie_rod_local
    start_point = Point3f(summary.reference_lever_length_mm, 0.0, 0.0)
    end_point = Point3f((summary.reference_lever_length_mm .+ tie_rod_plot_vector[1]), tie_rod_plot_vector[2], tie_rod_plot_vector[3])

    push!(plot_segments, start_point)
    push!(plot_segments, end_point)
    push!(plot_endpoints, end_point)
    push!(plot_angles, geometry.angle_deg)
    summary.plotted_count += 1

    return nothing
end

"""
    tie_rod_rotational_component_angle_sweep(phi_limits_deg, steering, suspension; side = :Left, ...)

Sweeps phi_x, phi_z and symmetric compression while phi_y is fixed.
The rotational component lever/tie rod angle is evaluated at the shared ball joint:

- rotational component lever vector: ball joint -> rotational component lever mount
- tie rod vector: ball joint -> steering sphere joint

For the 3D plot the rotational component lever is kept fixed in a local coordinate system:

- origin at the rotational component lever mount
- x-axis along the current rotational component lever, from mount to ball joint
- z-axis is the vehicle z-axis projected orthogonal to the current x-axis

At phi_z = 0 deg and 30% compression this is the requested reference
coordinate system. The plot therefore shows the tie-rod direction envelope relative
to the rotational component lever. For symmetric vehicles one side is sufficient; `side = :Left`
is used by default.
"""
function tie_rod_rotational_component_angle_sweep(phi_limits_deg,
                                         steering,
                                         suspension;
                                         side = :Left,
                                         phi_x_range_deg = (0.0, phi_limits_deg[1]),
                                         phi_y_deg = phi_limits_deg[2],
                                         phi_z_range_deg = (-phi_limits_deg[3], phi_limits_deg[3]),
                                         compression_range_percent = (0.0, 100.0),
                                         high_steering_compression_range_percent = compression_range_percent,
                                         high_steering_phi_z_threshold_deg = 15.0,
                                         angle_step_deg = 1.0,
                                         compression_step_percent = 1.0,
                                         reference_compression_percent = 30.0,
                                         max_plot_vectors_per_side = 3000,
                                         display_tie_rod_length_mm = 70.0,
                                         normalize_tie_rod_vectors_for_plot = true,
                                         progress_every = 100_000)
    phi_x_values = sweep_values(phi_x_range_deg[1], phi_x_range_deg[2], angle_step_deg)
    phi_z_values = sweep_values(phi_z_range_deg[1], phi_z_range_deg[2], angle_step_deg)
    default_compression_values = sweep_values(compression_range_percent[1], compression_range_percent[2], compression_step_percent)
    high_steering_compression_values = sweep_values(
        high_steering_compression_range_percent[1],
        high_steering_compression_range_percent[2],
        compression_step_percent,
    )

    total_pose_count = sum(
        length(compression_values_for_phi_z(
            phi_z,
            default_compression_values,
            high_steering_compression_values;
            high_steering_phi_z_threshold_deg = high_steering_phi_z_threshold_deg,
        )) for _ in phi_x_values, phi_z in phi_z_values
    )
    plot_stride = max(1, ceil(Int, total_pose_count / max_plot_vectors_per_side))

    side_index = side_index_from_symbol(side)
    reference = reference_geometry(
        phi_y_deg,
        steering,
        suspension,
        side_index;
        reference_compression_percent = reference_compression_percent,
    )
    summary = TieRodRotationalComponentSummary(
        side,
        side_index,
        tie_rod_rotational_component_unit(reference.tie_rod_local),
        reference.reference_lever_length_mm,
    )
    plot_segments = Point3f[]
    plot_endpoints = Point3f[]
    plot_angles = Float64[]

    steering_work = deepcopy(steering)
    suspension_work = deepcopy(suspension)
    pose_index = 0

    @printf("Sweeping %d poses (%d phi_x * %d phi_z with variable symmetric compression ranges)\n",
            total_pose_count,
            length(phi_x_values),
            length(phi_z_values))
    @printf("Using %.1f..%.1f %% compression for |phi_z| <= %.1f deg, %.1f..%.1f %% for larger |phi_z|\n",
            compression_range_percent[1],
            compression_range_percent[2],
            high_steering_phi_z_threshold_deg,
            high_steering_compression_range_percent[1],
            high_steering_compression_range_percent[2])
    @printf("Plotting every %d pose(s), up to about %d vectors per side\n", plot_stride, max_plot_vectors_per_side)

    for phi_x in phi_x_values, phi_z in phi_z_values
        compression_values = compression_values_for_phi_z(
            phi_z,
            default_compression_values,
            high_steering_compression_values;
            high_steering_phi_z_threshold_deg = high_steering_phi_z_threshold_deg,
        )

        for compression in compression_values
            pose_index += 1

            suspension_work.damper[1].compression = compression
            suspension_work.damper[2].compression = compression

            try
                micromobilitykinematics.update!((phi_x, phi_y_deg, phi_z), steering_work, suspension_work)

                geometry = tie_rod_rotational_component_geometry(steering_work, side_index)
                pose = pose_row(phi_x, phi_y_deg, phi_z, suspension_work, side)
                update_summary!(summary, geometry, pose)

                if pose_index % plot_stride == 0
                    add_plot_vector!(
                        plot_segments,
                        plot_endpoints,
                        plot_angles,
                        summary,
                        geometry;
                        display_tie_rod_length_mm = display_tie_rod_length_mm,
                        normalize_tie_rod_vectors_for_plot = normalize_tie_rod_vectors_for_plot,
                    )
                end
            catch err
                pose = failed_pose_row(phi_x, phi_y_deg, phi_z, compression, side)
                reason = kinematics_failure_reason(err, steering_work, suspension_work)
                record_failure!(summary, pose, reason)
                steering_work = deepcopy(steering)
                suspension_work = deepcopy(suspension)
            end

            if progress_every > 0 && pose_index % progress_every == 0
                @printf("  %d / %d poses\n", pose_index, total_pose_count)
                flush(stdout)
            end
        end
    end

    finalize_summary!(summary)

    return (
        summary = summary,
        plot_segments = plot_segments,
        plot_endpoints = plot_endpoints,
        plot_angles = plot_angles,
        total_pose_count = total_pose_count,
        plot_stride = plot_stride,
    )
end

function print_tie_rod_rotational_component_summary(result)
    println()
    println("Tie rod / rotational component lever angle range")

    summary = result.summary
    mean_direction = tie_rod_rotational_component_unit(summary.direction_sum)

    @printf(
        "%s: link angle %.3f deg .. %.3f deg | max deviation from neutral tie-rod direction %.3f deg | evaluated %d, failed %d, plotted %d\n",
        string(summary.side),
        summary.min_angle_deg,
        summary.max_angle_deg,
        summary.max_reference_direction_deviation_deg,
        summary.evaluated_count,
        summary.failed_count,
        summary.plotted_count,
    )
    println("  min link-angle pose: ", summary.min_angle_pose)
    println("  max link-angle pose: ", summary.max_angle_pose)
    println("  max deviation from neutral pose: ", summary.max_reference_direction_deviation_pose)
    println("  worst pose for optimal joint axis: ", summary.optimal_required_pose)
    @printf(
        "  mean tie-rod direction in local rotational-component frame: [%.4f, %.4f, %.4f]\n",
        mean_direction[1],
        mean_direction[2],
        mean_direction[3],
    )
    @printf(
        "  optimal joint neutral axis in local rotational-component frame: [%.4f, %.4f, %.4f]\n",
        summary.optimal_direction[1],
        summary.optimal_direction[2],
        summary.optimal_direction[3],
    )
    @printf(
        "  required ball-joint articulation around optimal axis: +/- %.3f deg\n",
        summary.optimal_required_angle_deg,
    )
    @printf(
        "  signed component angles around optimal axis: lateral %.3f .. %.3f deg | vertical z %.3f .. %.3f deg\n",
        summary.optimal_lateral_min_deg,
        summary.optimal_lateral_max_deg,
        summary.optimal_vertical_min_deg,
        summary.optimal_vertical_max_deg,
    )
    @printf(
        "  optimal axis offset from 0 deg / 30%% compression tie-rod direction: %.3f deg\n",
        summary.optimal_reference_offset_deg,
    )

    print_failed_state_summary(summary)
end

function field_extrema(rows, field)
    values = [getproperty(row, field) for row in rows]
    return extrema(values)
end

function value_counts(rows, field)
    counts = Dict{Float64,Int}()

    for row in rows
        value = Float64(getproperty(row, field))
        counts[value] = get(counts, value, 0) + 1
    end

    return counts
end

function print_top_counts(counts, label; n = 8)
    pairs = sort(collect(counts); by = pair -> (-pair.second, pair.first))
    isempty(pairs) && return nothing

    print("  most failed $label: ")

    for (index, (value, count)) in enumerate(first(pairs, min(n, length(pairs))))
        index > 1 && print(", ")
        @printf("%.1f (%d)", value, count)
    end

    println()
    return nothing
end

function print_min_failed_compression_by_field(rows, field, label; n = 16)
    cutoffs = Dict{Float64,Float64}()

    for row in rows
        key = Float64(getproperty(row, field))
        compression = Float64(row.compression_percent)
        cutoffs[key] = haskey(cutoffs, key) ? min(cutoffs[key], compression) : compression
    end

    pairs = sort(collect(cutoffs); by = pair -> pair.first)
    isempty(pairs) && return nothing

    println("  minimum failed compression by $label:")

    if length(pairs) <= n
        for (key, compression) in pairs
            @printf("    %.1f -> %.1f %%\n", key, compression)
        end
    else
        for (key, compression) in pairs[1:div(n, 2)]
            @printf("    %.1f -> %.1f %%\n", key, compression)
        end

        println("    ...")

        for (key, compression) in pairs[end-div(n, 2)+1:end]
            @printf("    %.1f -> %.1f %%\n", key, compression)
        end
    end

    return nothing
end

function print_failed_state_summary(summary)
    println()
    println("Failed-state summary")

    if isempty(summary.failed_poses)
        println("  no failed states")
        return nothing
    end

    phi_x_min, phi_x_max = field_extrema(summary.failed_poses, :phi_x_deg)
    phi_z_min, phi_z_max = field_extrema(summary.failed_poses, :phi_z_deg)
    compression_min, compression_max = field_extrema(summary.failed_poses, :compression_percent)

    @printf(
        "  failed poses: %d\n  phi_x range: %.1f .. %.1f deg\n  phi_z range: %.1f .. %.1f deg\n  compression range: %.1f .. %.1f %%\n",
        summary.failed_count,
        phi_x_min,
        phi_x_max,
        phi_z_min,
        phi_z_max,
        compression_min,
        compression_max,
    )

    print_top_counts(value_counts(summary.failed_poses, :compression_percent), "compression values")
    print_top_counts(value_counts(summary.failed_poses, :phi_z_deg), "phi_z values")
    print_top_counts(value_counts(summary.failed_poses, :phi_x_deg), "phi_x values")
    print_min_failed_compression_by_field(summary.failed_poses, :phi_z_deg, "phi_z")
    print_min_failed_compression_by_field(summary.failed_poses, :phi_x_deg, "phi_x")

    println("  first failed pose: ", first(summary.failed_poses))
    println("  last failed pose: ", last(summary.failed_poses))

    println("  failure reasons:")
    reason_pairs = sort(collect(summary.failed_reason_counts); by = pair -> -pair.second)

    for (reason, count) in first(reason_pairs, min(5, length(reason_pairs)))
        println("    $count x $reason")
    end

    return nothing
end

function format_pose_for_label(pose)
    pose === nothing && return "n/a"

    return @sprintf(
        "phi_x %.1f deg, phi_z %.1f deg, compression %.1f%%",
        pose.phi_x_deg,
        pose.phi_z_deg,
        pose.compression_percent,
    )
end

function plot_info_text(summary)
    return @sprintf(
        """
        Construction values (%s side)

        Optimal joint neutral axis
        local [x, y, z] = [%.4f, %.4f, %.4f]

        Required articulation
        +/- %.2f deg from optimal axis

        Component angle ranges
        lateral: %.2f .. %.2f deg
        vertical z: %.2f .. %.2f deg

        Axis offset from reference
        %.2f deg vs. 0 deg steering / 30%% compression

        Tie rod / rotational component lever angle
        %.2f deg .. %.2f deg

        Worst pose for optimal axis
        %s

        Evaluated / failed states
        %d / %d
        """,
        string(summary.side),
        summary.optimal_direction[1],
        summary.optimal_direction[2],
        summary.optimal_direction[3],
        summary.optimal_required_angle_deg,
        summary.optimal_lateral_min_deg,
        summary.optimal_lateral_max_deg,
        summary.optimal_vertical_min_deg,
        summary.optimal_vertical_max_deg,
        summary.optimal_reference_offset_deg,
        summary.min_angle_deg,
        summary.max_angle_deg,
        format_pose_for_label(summary.optimal_required_pose),
        summary.evaluated_count,
        summary.failed_count,
    )
end

function component_angle_vectors(summary)
    basis = optimal_axis_component_basis(summary.optimal_direction)
    lateral_angles = Float64[]
    vertical_angles = Float64[]

    for direction in summary.directions
        component_angles = direction_component_angles_deg(direction, basis)
        push!(lateral_angles, component_angles.lateral)
        push!(vertical_angles, component_angles.vertical)
    end

    return lateral_angles, vertical_angles
end

function plot_tie_rod_rotational_component_vectors(result;
                                          save_path = joinpath(@__DIR__, "tie_rod_rotational_component_angle_range.png"),
                                          title = "Tie rod direction envelope in rotational-component-local frame")
    fig = Figure(size = (1500, 760))
    summary = result.summary
    all_angles = result.plot_angles
    colorrange = isempty(all_angles) ? (0.0, 1.0) : extrema(all_angles)

    ax = Axis3(
        fig[1:2, 1],
        xlabel = "local x along rotational component lever [mm]",
        ylabel = "local y [mm]",
        zlabel = "local z, vehicle-up projection [mm]",
        title = "$(summary.side) side",
        aspect = :data,
    )

    reference_start = Point3f(0.0, 0.0, 0.0)
    reference_end = Point3f(summary.reference_lever_length_mm, 0.0, 0.0)
    lines!(ax, [reference_start, reference_end]; color = :black, linewidth = 5)
    scatter!(ax, [reference_start, reference_end]; color = :black, markersize = 10)

    if !isempty(result.plot_segments)
        linesegments!(
            ax,
            result.plot_segments;
            color = :royalblue,
            linewidth = 1,
            transparency = true,
            alpha = 0.28,
        )
        scatter!(
            ax,
            result.plot_endpoints;
            color = result.plot_angles,
            colormap = :viridis,
            colorrange = colorrange,
            markersize = 4,
        )
    end

    optimal_vector_length = 100.0
    optimal_end = Point3f(
        summary.reference_lever_length_mm + summary.optimal_direction[1] * optimal_vector_length,
        summary.optimal_direction[2] * optimal_vector_length,
        summary.optimal_direction[3] * optimal_vector_length,
    )
    lines!(ax, [reference_end, optimal_end]; color = :red, linewidth = 6)
    scatter!(ax, [optimal_end]; color = :red, markersize = 12)

    lateral_angles, vertical_angles = component_angle_vectors(summary)
    component_axis = Axis(
        fig[1, 3],
        xlabel = "lateral angle from optimal axis [deg]",
        ylabel = "vertical z angle from optimal axis [deg]",
        title = "Angular envelope around optimal axis",
        aspect = DataAspect(),
    )
    scatter!(
        component_axis,
        lateral_angles,
        vertical_angles;
        color = (:royalblue, 0.22),
        markersize = 3,
    )
    vlines!(component_axis, [0.0]; color = :gray45, linewidth = 1)
    hlines!(component_axis, [0.0]; color = :gray45, linewidth = 1)
    scatter!(component_axis, [0.0], [0.0]; color = :red, markersize = 10)

    Label(fig[0, 1:3], title)
    Colorbar(fig[1:2, 2], colormap = :viridis, limits = colorrange, label = "angle between tie rod and rotational component lever [deg]")
    Label(
        fig[2, 3],
        plot_info_text(summary);
        justification = :left,
        tellheight = false,
        tellwidth = true,
        halign = :left,
        valign = :top,
    )

    save(save_path, fig)
    println()
    println("Saved plot: $save_path")

    return fig
end

function run_tie_rod_rotational_component_angle_example()
    phi_limits_deg = (15.0, 1.0, 35.0)
    phi_x_max_deg = 10.0
    angle_step_deg = 1.0
    compression_step_percent = 1.0

    steering_to_evaluate = isdefined(Main, :steering) ? deepcopy(Main.steering) : tie_rod_rotational_component_example_steering()
    suspension_to_evaluate = isdefined(Main, :suspension) ? deepcopy(Main.suspension) : tie_rod_rotational_component_example_suspension()

    result = tie_rod_rotational_component_angle_sweep(
        phi_limits_deg,
        steering_to_evaluate,
        suspension_to_evaluate;
        side = :Left,
        phi_x_range_deg = (0.0, phi_x_max_deg),
        phi_y_deg = phi_limits_deg[2],
        phi_z_range_deg = (-phi_limits_deg[3], phi_limits_deg[3]),
        compression_range_percent = (10.0, 90.0),
        high_steering_compression_range_percent = (20.0, 70.0),
        high_steering_phi_z_threshold_deg = 15.0,
        angle_step_deg = angle_step_deg,
        compression_step_percent = compression_step_percent,
        reference_compression_percent = 30.0,
        max_plot_vectors_per_side = 3000,
    )

    print_tie_rod_rotational_component_summary(result)
    fig = plot_tie_rod_rotational_component_vectors(result)

    return result, fig
end

result, fig = run_tie_rod_rotational_component_angle_example()
