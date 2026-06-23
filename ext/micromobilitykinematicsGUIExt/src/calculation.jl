"""
    ackermannratio(θ::Tuple{T,T,T}, 
                        chassis::Chassis, 
                        steering::Steering, 
                        suspension::Suspension) where {T >: Any}


Calculates the Ackermann steering ratio [%] based on the current steering configuration.
A result of 100% indicates a perfect match with ideal Ackermann behavior, while lower values indicate deviation due to geometry, articulation, or linkage constraints.

# Arguments
- `θ::Tuple{T,T,T}`: A tuple `(θx, θy, θz)` representing the current steering angles.
- `chassis::Chassis`: The chassis object containing structural vehicle information such as wheelbase.
- `steering::Steering`: The steering system configuration, used to evaluate current joint and linkage positions.
- `suspension::Suspension`: The suspension model affecting the geometry during steering.

# Description
This function evaluates how closely the current steering geometry approximates ideal Ackermann steering. It works by:
- Computing the `ackermann_deviation`, representing deviation from ideal geometry.
- Retrieving the effective wheelbase from the `Measurements` helper struct.
- Using these values to compute the ratio:
        AckermannRatio = (wheel_base / (wheel_base + objective)) * 100
- A result of 100% indicates a perfect match with ideal Ackermann behavior, while lower values indicate deviation due to geometry, articulation, or linkage constraints.

# Returns 
- Float64: The computed Ackermann ratio in percent [%].
"""
function ackermannratio(θ::Tuple{T,T,T}, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension;
                            signed = ackermann_ratio_signed()) where {T >: Any}
    #wheel_offset                    # Distance Rotationspoint and Wheelcenter
    #offset = wheel_offset * sind(δo)

    measurment = Measurements(chassis, steering)
    deviation = ackermann_deviation(θ, chassis, steering, suspension)

    objective = abs(deviation)
    L = objective + measurment.wheel_base #+ offset
    ratio = (measurment.wheel_base/L)*100

    signed || return ratio

    return deviation < 0.0 ? 200.0 - ratio : ratio

end 


"""
    turning_radius(chassis::Chassis, steering::Steering)

Computes the turning radius in millimeters based on the current outer wheel steering angle and the wheelbase.

# Arguments:
- `chassis::Chassis`: The vehicle chassis object, providing the wheelbase.
- `steering::Steering`: The steering configuration, including the outer wheel steering angle δo (in degrees).

# Description:
This function calculates the turning radius of the vehicle assuming a circular path defined by the outer wheel.
The formula used is:

    radius = wheel_base / sin(δo)

where δo is converted from degrees to radians.

If δo is equal to 0.0 (i.e., no steering input), the function returns NaN since a turning radius cannot be defined.

# Returns:
- Float64: The calculated turning radius in millimeters, or NaN if δo == 0.0.
"""
function turning_radius(chassis::Chassis, steering::Steering)

    if steering.δo == 0.0
        return NaN
    end

    measurment = Measurements(chassis, steering)
    δo = deg2rad(steering.δo)

    radius = measurment.wheel_base / sin(δo)

    return radius
end

"""
    steering_radii(chassis::Chassis, 
                   steering::Steering, 
                   suspension::Suspension, 
                   θ_max::Tuple{T,T,T}; 
                   step_size = 1) where {T <: Any}

Computes a matrix of turning radii over a grid of steering angles (θx, θz), with θy held constant.

Arguments:
- `chassis::Chassis`: The vehicle's chassis model, providing wheelbase information.
- `steering::Steering`: The steering system, used to determine steering angles.
- `suspension::Suspension`: The suspension system affecting the steering geometry.
- `θ_max::Tuple{T,T,T}`: Maximum angle values (θx_max, θy_fixed, θz_max) defining the steering input space.

# Keywords
- `step_size`: Step size for the θx and θz sweep. Defaults to 1 degree.

# Description:
This function creates a 2D grid over the (θx, θz) angle space and computes the turning radius for each combination.
For each angle configuration:
- The steering system is updated via `update!`.
- The outer wheel angle δo is used to compute the radius as:

      radius = wheel_base / sin(δo)

- If δo is 0.0, the radius is set to NaN.

# Returns:
- A 2D Array of Float64 values representing turning radii in millimeters across the θx–θz angle grid.
"""
function steering_radii(chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension, 
                            θ_max::Tuple{T,T,T}; 
                            step_size = 1 ) where {T <: Any}

    θx_max , θy, θz_max = θ_max
    θ_matrix = [(θx, θy, θz) for θx in 0:step_size:θx_max, θz in 0:step_size:θz_max]
    radii = [ 0.0 for x in 0:step_size:θx_max, z in 0:step_size:θz_max]


    for θ in θ_matrix
        θx, θy, θz = θ
        MMK.update!(θ, steering, suspension)

        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        if steering.δo == 0.0
            radii[θx_i+1,θz_i+1] = NaN
        else
    
        measurment = Measurements(chassis, steering)
        δo = deg2rad(steering.δo)
    
        radii[θx_i + 1,θz_i + 1] = measurment.wheel_base / sin(δo)
        end
    end 

    return radii

end

"""
    steering_radii_θz(θx::T, θy::T, θz_max::T, 
                      chassis::Chassis, 
                      steering::Steering, 
                      suspension::Suspension; 
                      step_size = 1) where {T <: Any}

Computes a list of turning radii for a sweep of θz (inner wheel angle), with fixed θx and θy.

# Arguments:
- `θx::T`: Steering angle around the x-axis (fixed).
- `θy::T`: Steering angle around the y-axis (fixed).
- `θz_max::T`: Maximum steering angle to sweep over θz (in degrees).
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system, providing access to current δo (outer steering angle).
- `suspension::Suspension`: The suspension model used in geometry update.

# Keywords
- `step_size`: Angle increment for θz sweep. Defaults to 1 degree.

Description:
This function iterates over the range `0:step_size:θz_max`, updating the steering and suspension system
for each θz value (with θx and θy held constant). It computes the turning radius using:

    radius = wheel_base / sin(δo)

If `δo == 0.0`, the function returns `NaN` for that configuration.

# Returns:
- A 1D Array of Float64 values representing turning radii [mm] across the θz angle sweep.
"""
function steering_radii_θz(θx::T,
                            θy::T,
                            θz_max::T, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension; 
                            step_size = 1 ) where {T <: Any}

    θ_matrix = [i for i in 0:step_size:θz_max]
    radii = []


    for θ in θ_matrix
        θz = θ
        MMK.update!((θx, θy, θz), steering, suspension)

        if steering.δo == 0.0
            push!(radii,NaN)
        else
            measurment = Measurements(chassis, steering)
            δo = deg2rad(steering.δo)
            
            radius = measurment.wheel_base / sin(δo)
            push!(radii, radius)
        end
    end 
    return radii

end


"""
    ackermannratio_θz(θx::T, θy::T, θz_max::T,
                      chassis::Chassis,
                      steering::Steering,
                      suspension::Suspension;
                      step_size = 1) where {T <: Any}

Computes the Ackermann ratio [%] across a sweep of θz values, with fixed θx and θy angles.

# Arguments:
- `θx::T`: Fixed steering angle around the x-axis.
- `θy::T`: Fixed steering angle around the y-axis.
- `θz_max::T`: Maximum value for θz (inner wheel angle in degrees).
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system used to retrieve joint states and δo.
- `suspension::Suspension`: The suspension system affecting wheel geometry.

# Keywords
- `step_size`: Step size for the θz sweep (in degrees). Defaults to 1.

# Description:
This function sweeps θz from 0 to `θz_max` in the specified step size and computes the Ackermann ratio
at each step using the current steering and suspension configuration.

If the outer wheel angle `δo` is 0.0 (undefined steering geometry), the function substitutes `θz + 1` to avoid division by zero.

# Returns:
- A 1D Array of Float64 values representing the Ackermann ratio [%] across the θz sweep.
"""
function ackermannratio_θz(θx::T, 
                            θy::T, 
                            θz_max::T, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension; 
                            signed = ackermann_ratio_signed(),
                            step_size = 1 ) where {T <: Any}

    θ_matrix = [i for i in 0:step_size:θz_max]
    ratio = []


    for θ in θ_matrix
        θz = θ
        MMK.update!((θx, θy, θz), steering, suspension)



        if steering.δo == 0.0
            push!(ratio,ackermannratio((θx, θy, θz+1.0),chassis, steering, suspension; signed = signed))
        else
            push!(ratio,ackermannratio((θx, θy, θz),chassis, steering, suspension; signed = signed))
        end
    end 
    return ratio

end

function ackermannratio_θx(θx_max::T, 
                            θy::T, 
                            θz::T, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension; 
                            signed = ackermann_ratio_signed(),
                            step_size = 1 ) where {T <: Any}

    θ_matrix = [i for i in 0:step_size:θx_max]
    ratio = []

    for θ in θ_matrix
        θx = θ
        MMK.update!((θx, θy, θz), steering, suspension)

        if steering.δo == 0.0
            push!(ratio, NaN)
        else
            push!(ratio, ackermannratio((θx, θy, θz), chassis, steering, suspension; signed = signed))
        end
    end

    return ratio
end


const ACKERMANN_RATIO_SIGNED = Ref(false)

ackermann_ratio_signed() = ACKERMANN_RATIO_SIGNED[]
set_ackermann_ratio_signed!(value::Bool) = (ACKERMANN_RATIO_SIGNED[] = value)

"""
    ackermannratio_surface(chassis::Chassis, 
                           steering::Steering, 
                           suspension::Suspension, 
                           θ_max::Tuple{T,T,T}; 
                           step_size = 1) where {T <: Any}

Computes a 2D surface of Ackermann ratio [%] values over a grid of (θx, θz) steering angles, with θy fixed.

# Arguments:
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system model containing joint and linkage data.
- `suspension::Suspension`: The suspension system that affects wheel positioning.
- `θ_max::Tuple{T,T,T}`: Tuple `(θx_max, θy_fixed, θz_max)` specifying angle sweep limits.

# Keywords
- `step_size`: Increment size for the θx and θz sweep. Default is 1 degree.

# Description:
This function creates a 2D grid of angle configurations for θx and θz (holding θy constant) and computes
the Ackermann ratio at each grid point using the current steering and suspension configuration.

If the outer wheel steering angle `δo` is zero, the corresponding ratio value is set to NaN to avoid invalid computation.

# Returns:
- A 2D Array of Float64 values representing the Ackermann ratio [%] across the θx–θz parameter space.
"""
function ackermannratio_surface(chassis::Chassis, 
                                    steering::Steering, 
                                    suspension::Suspension, 
                                    θ_max::Tuple{T,T,T};
                                    signed = ackermann_ratio_signed(),
                                    step_size = 1 ) where {T <: Any}

    θx_max , θy, θz_max = θ_max
    θ_matrix = [(θx, θy, θz) for θx in 0.0:step_size:θx_max, θz in 0.0:step_size:θz_max]
    ratio = [ 0.0 for x in 0:step_size:θx_max, z in 0:step_size:θz_max]

    for θ in θ_matrix
        θx, θy, θz = θ
        MMK.update!(θ, steering, suspension)

        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        if steering.δo == 0.0
            ratio[θx_i+1,θz_i+1] = NaN
        else
    
        ratio[θx_i+1,θz_i+1] = ackermannratio(θ,chassis, steering, suspension; signed = signed)
        end
    end 
    return ratio
end

function ackermann_deviation_θz(θx::T, 
                                    θy::T, 
                                    θz_max::T, 
                                    chassis::Chassis, 
                                    steering::Steering, 
                                    suspension::Suspension; 
                                    step_size = 1 ) where {T <: Any}

    θ_matrix = [i for i in 0:step_size:θz_max]
    deviation = []


    for θ in θ_matrix
        θz = θ
        MMK.update!((θx, θy, θz), steering, suspension)

        if steering.δo == 0.0
            push!(deviation,ackermann_deviation((θx, θy, θz+1.0),chassis, steering, suspension))
        else
            push!(deviation,ackermann_deviation((θx, θy, θz),chassis, steering, suspension))
        end
    end 
    return deviation

end


function ackermann_deviation_surface(chassis::Chassis, 
                                        steering::Steering, 
                                        suspension::Suspension, 
                                        θ_max::Tuple{T,T,T};
                                        step_size = 1 ) where {T <: Any}

    θx_max , θy, θz_max = θ_max
    θ_matrix = [(θx, θy, θz) for θx in 0.0:step_size:θx_max, θz in 0.0:step_size:θz_max]
    deviation = [ 0.0 for x in 0:step_size:θx_max, z in 0:step_size:θz_max]

    for θ in θ_matrix
        θx, θy, θz = θ
        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        try
            MMK.update!(θ, steering, suspension)

            if steering.δo == 0.0
                deviation[θx_i+1,θz_i+1] = NaN
            else
                deviation[θx_i+1,θz_i+1] = ackermann_deviation(θ,chassis, steering, suspension)
            end
        catch
            deviation[θx_i+1,θz_i+1] = NaN
        end
    end 
    return deviation
end



"""
    ax_θ_vs_δi(steering::Steering, θ_max::Tuple{T,T,T}; step_size = 1) where {T <: Any}

Generates a matrix of inner wheel steering angles (`δi`) over a grid of steering input angles.

# Arguments
- `steering`: The `Steering` object, which provides access to current kinematic state and computes updated angles.
- `θ_max`: A tuple `(θx_max, θy, θz_max)` defining the maximum steering angles to sample.
- `step_size`: Optional; step resolution for θx and θz sampling (default: `1`).

# Description
This function:
- Constructs a grid of steering input angles `(θx, θy, θz)` where `θx` and `θz` are varied over their ranges.
- Calls `update!` for each angle combination to compute the steering state.
- Extracts the inner wheel angle `δi` from the `steering` object and stores it in a 2D matrix.
- Handles the case where `δo == 0.0` by inserting `NaN` to avoid divide-by-zero or undefined states.

This function is typically used to prepare surface data for plotting steering behavior, e.g., in `ax_θ_vs_δ_plot!`.

# Returns
A 2D array (`Matrix{Float64}`) of computed inner wheel angles `δi` indexed by discretized `θx` and `θz`.
"""
function ax_θ_vs_δi(steering::Steering,
                        suspension::Suspension, 
                        θ_max::Tuple{T,T,T};
                        step_size = 1 ) where {T <: Any}

    θx_max , θy, θz_max = θ_max
    θ_matrix = [(θx, θy, θz) for θx in 0.0:step_size:θx_max, θz in 0.0:step_size:θz_max]
    δi = [ 0.0 for x in 0:step_size:θx_max, z in 0:step_size:θz_max]

    for θ in θ_matrix
        θx, θy, θz = θ
        MMK.update!(θ, steering, suspension)

        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        if steering.δo == 0.0
            δi[θx_i+1,θz_i+1] = NaN
        else
    
        δi[θx_i+1,θz_i+1] = steering.δi
        end
    end 
    return δi
end


"""
    ax_θ_vs_δo(steering::Steering, θ_max::Tuple{T,T,T}; step_size = 1) where {T <: Any}

Generates a matrix of outer wheel steering angles (`δo`) over a grid of steering input angles.

# Arguments
- `steering`: The `Steering` object, which holds the kinematic model and computes steering responses.
- `θ_max`: A tuple `(θx_max, θy, θz_max)` defining the maximum values for the steering input angles.
- `step_size`: Optional; resolution of the sampling grid for `θx` and `θz` (default: `1`).

# Description
This function:
- Constructs a 2D parameter grid of steering angles `(θx, θy, θz)`, keeping `θy` fixed.
- Iteratively calls `update!` on the `steering` object for each angle combination.
- Records the outer wheel steering angle `δo` into a matrix indexed by `θx` and `θz`.
- Fills matrix entries with `NaN` when `δo` is zero, signaling invalid or unresponsive configurations.

The resulting matrix is used for visualization of steering behavior in 3D surface plots (e.g., `ax_θ_vs_δ_plot!`).

# Returns
A 2D array (`Matrix{Float64}`) containing outer wheel angles `δo` over the sampled input range.
"""
function ax_θ_vs_δo(steering::Steering,
                        suspension::Suspension, 
                        θ_max::Tuple{T,T,T};
                        step_size = 1 ) where {T <: Any}

    θx_max , θy, θz_max = θ_max
    θ_matrix = [(θx, θy, θz) for θx in 0.0:step_size:θx_max, θz in 0.0:step_size:θz_max]
    δo = [ 0.0 for x in 0:step_size:θx_max, z in 0:step_size:θz_max]

    for θ in θ_matrix
        θx, θy, θz = θ
        MMK.update!(θ, steering, suspension)

        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        if steering.δo == 0.0
            δo[θx_i+1,θz_i+1] = NaN
        else
    
        δo[θx_i+1,θz_i+1] = steering.δo
        end
    end 
    return δo
end



function compr_vs_δ(θ::Tuple{T,T,T},
                        steering::Steering,
                        suspension::Suspension;
                        step_size = 1 ) where {T <: Any}

    compression_values = collect(0.0:step_size:100.0)
    last(compression_values) == 100.0 || push!(compression_values, 100.0)
    δi = fill(NaN, length(compression_values), length(compression_values))
    δo = fill(NaN, length(compression_values), length(compression_values))

    for (l_index, l_compr) in enumerate(compression_values), (r_index, r_compr) in enumerate(compression_values)
        suspension.damper[1].compression = l_compr
        suspension.damper[2].compression = r_compr

        try
            MMK.update!(θ, steering, suspension)

            δi[l_index,r_index] = steering.δi
            δo[l_index,r_index] = steering.δo
        catch err
            δi[l_index,r_index] = NaN
            δo[l_index,r_index] = NaN
        end
    end 
    
    return δi, δo
end

function left_wheel_delta_vs_compression_θz(θx::T,
                                            θy::T,
                                            θz_max::T,
                                            steering::Steering,
                                            suspension::Suspension;
                                            fixed_right_compression = suspension.damper[2].compression,
                                            step_size = 1) where {T <: Any}

    compression_values = collect(0.0:step_size:100.0)
    last(compression_values) == 100.0 || push!(compression_values, 100.0)

    θz_values = collect(0.0:step_size:θz_max)
    last(θz_values) == θz_max || push!(θz_values, θz_max)

    delta_left = fill(NaN, length(compression_values), length(θz_values))

    for (compression_index, left_compression) in enumerate(compression_values), (θz_index, θz) in enumerate(θz_values)
        suspension.damper[1].compression = left_compression
        suspension.damper[2].compression = fixed_right_compression

        try
            MMK.update!((θx, θy, 0.0), steering, suspension)
            baseline_left_angle = steering.δo

            MMK.update!((θx, θy, θz), steering, suspension)
            current_left_angle = steering.δo

            delta_left[compression_index, θz_index] = current_left_angle - baseline_left_angle
        catch
            delta_left[compression_index, θz_index] = NaN
        end
    end

    return delta_left
end

function compression_sweep_values(; step_size = 1.0)
    values = collect(0.0:step_size:100.0)
    last(values) == 100.0 || push!(values, 100.0)
    return values
end

function finite_norm(values)
    return sqrt(sum(value -> Float64(value)^2, values))
end

function clean_error_info_copy(instance)
    if !hasproperty(instance, :err_info)
        return deepcopy(instance)
    end

    original_error_info = instance.err_info
    fresh_error_info = original_error_info === nothing ? MMK.ErrorInfo() : typeof(original_error_info)()

    try
        instance.err_info = fresh_error_info
        return deepcopy(instance)
    finally
        instance.err_info = original_error_info
    end
end

function wheel_center_vehicle_position(steering::Steering, suspension::Suspension, side_index::Int)
    lower_joint = Float64.(suspension.lowerwishbone[side_index].sphere_joint)
    upper_joint = Float64.(suspension.upperwishbone[side_index].sphere_joint)
    wheel_axis_z = upper_joint .- lower_joint
    wheel_axis_z ./= finite_norm(wheel_axis_z)

    base_vec_y, base_vec_x, base_vec_z = MMK.calc_basis_vectors(wheel_axis_z)
    wheel_basis = [base_vec_x base_vec_y base_vec_z]
    wheel_offset = [
        suspension.wheelmount.offset_x,
        suspension.wheelmount.offset_y,
        suspension.wheelmount.offset_z,
    ]

    wheel_center_local = lower_joint .+ wheel_basis * wheel_offset
    side_index == 2 && (wheel_center_local = wheel_center_local .* [1.0, -1.0, 1.0])

    return Float64.(steering.wishbone_ucs_position[side_index]) .+ wheel_center_local
end

function lower_joint_vehicle_position(steering::Steering, suspension::Suspension, side_index::Int)
    lower_joint = Float64.(suspension.lowerwishbone[side_index].sphere_joint)
    side_index == 2 && (lower_joint = lower_joint .* [1.0, -1.0, 1.0])

    return Float64.(steering.wishbone_ucs_position[side_index]) .+ lower_joint
end

function rotate_vector_around_axis(vector, axis, angle)
    return vector .* cos(angle) .+
           cross(axis, vector) .* sin(angle) .+
           axis .* dot(axis, vector) .* (1.0 - cos(angle))
end

function wheel_axis_vehicle_direction(suspension::Suspension, side_index::Int)
    lower_joint = Float64.(suspension.lowerwishbone[side_index].sphere_joint)
    upper_joint = Float64.(suspension.upperwishbone[side_index].sphere_joint)
    wheel_axis_z = upper_joint .- lower_joint
    axis_length = finite_norm(wheel_axis_z)

    axis_length <= eps(Float64) && return [NaN, NaN, NaN]

    wheel_axis_z ./= axis_length
    side_index == 2 && (wheel_axis_z = wheel_axis_z .* [1.0, -1.0, 1.0])

    return wheel_axis_z
end

function wheel_camber_angle(suspension::Suspension, side_index::Int)
    wheel_axis_z = wheel_axis_vehicle_direction(suspension, side_index)
    any(isnan, wheel_axis_z) && return NaN

    return atand(wheel_axis_z[2], wheel_axis_z[3])
end

function steered_wheel_center_vehicle_position(steering::Steering, suspension::Suspension, side_index::Int)
    wheel_center = wheel_center_vehicle_position(steering, suspension, side_index)

    if steering.circle_joints === nothing ||
        steering.circle_joints_neutral === nothing ||
        steering.track_lever_mounting_points_ucs === nothing
        return wheel_center
    end

    axis = wheel_axis_vehicle_direction(suspension, side_index)
    any(isnan, axis) && return wheel_center

    track_lever_mount = Float64.(steering.track_lever_mounting_points_ucs[side_index])
    neutral_track_lever = Float64.(steering.circle_joints_neutral[side_index]) .- track_lever_mount
    moved_track_lever = Float64.(steering.circle_joints[side_index]) .- track_lever_mount

    if finite_norm(neutral_track_lever) <= eps(Float64) || finite_norm(moved_track_lever) <= eps(Float64)
        return wheel_center
    end

    steering_angle = atan(dot(axis, cross(neutral_track_lever, moved_track_lever)), dot(neutral_track_lever, moved_track_lever))
    lower_joint = lower_joint_vehicle_position(steering, suspension, side_index)
    wheel_center_offset = wheel_center .- lower_joint

    return lower_joint .+ rotate_vector_around_axis(wheel_center_offset, axis, steering_angle)
end

function wheel_center_path(steering::Steering,
                            suspension::Suspension;
                            step_size = 1.0,
                            symmetric = true)
    compression_values = compression_sweep_values(; step_size = step_size)
    left_path = Point3f[]
    right_path = Point3f[]
    suspension_copy = deepcopy(suspension)

    for compression in compression_values
        suspension_copy.damper[1].compression = compression
        suspension_copy.damper[2].compression = symmetric ? compression : suspension.damper[2].compression

        try
            MMK.suspensionkinematics!(suspension_copy)
            left_center = wheel_center_vehicle_position(steering, suspension_copy, 1)
            right_center = wheel_center_vehicle_position(steering, suspension_copy, 2)
            push!(left_path, Point3f(left_center...))
            push!(right_path, Point3f(right_center...))
        catch
            push!(left_path, Point3f(NaN, NaN, NaN))
            push!(right_path, Point3f(NaN, NaN, NaN))
        end
    end

    return compression_values, left_path, right_path
end

function sweep_values(max_value; step_size = 1.0)
    values = collect(0.0:step_size:max_value)
    last(values) == max_value || push!(values, max_value)
    return values
end

function signed_sweep_values(max_value; step_size = 1.0)
    values = collect(-max_value:step_size:max_value)
    append!(values, [-max_value, 0.0, max_value])
    return sort(unique(values))
end

function wheel_center_surface_coordinate_matrices(points)
    xs = [Float64(point[1]) for point in points]
    ys = [Float64(point[2]) for point in points]
    zs = [Float64(point[3]) for point in points]

    return xs, ys, zs
end

function wheel_center_surface(θx,
                                θy,
                                θz_max,
                                steering::Steering,
                                suspension::Suspension;
                                compression_step = 5.0,
                                θz_step = 2.0)
    compression_values = compression_sweep_values(; step_size = compression_step)
    θz_values = signed_sweep_values(θz_max; step_size = θz_step)

    left_points = fill(Point3f(NaN, NaN, NaN), length(compression_values), length(θz_values))
    right_points = fill(Point3f(NaN, NaN, NaN), length(compression_values), length(θz_values))

    for (compression_index, compression) in enumerate(compression_values), (θz_index, θz) in enumerate(θz_values)
        steering_copy = clean_error_info_copy(steering)
        suspension_copy = clean_error_info_copy(suspension)

        suspension_copy.damper[1].compression = compression
        suspension_copy.damper[2].compression = compression

        try
            MMK.update!((θx, θy, θz), steering_copy, suspension_copy)

            left_center = steered_wheel_center_vehicle_position(steering_copy, suspension_copy, 1)
            right_center = steered_wheel_center_vehicle_position(steering_copy, suspension_copy, 2)

            left_points[compression_index, θz_index] = Point3f(left_center...)
            right_points[compression_index, θz_index] = Point3f(right_center...)
        catch
            left_points[compression_index, θz_index] = Point3f(NaN, NaN, NaN)
            right_points[compression_index, θz_index] = Point3f(NaN, NaN, NaN)
        end
    end

    left_x, left_y, left_z = wheel_center_surface_coordinate_matrices(left_points)
    right_x, right_y, right_z = wheel_center_surface_coordinate_matrices(right_points)

    return compression_values, θz_values, left_x, left_y, left_z, right_x, right_y, right_z
end

function track_width_over_compression(steering::Steering,
                                        suspension::Suspension;
                                        step_size = 1.0)
    compression_values, left_path, right_path = wheel_center_path(steering, suspension; step_size = step_size)
    track_width = Float64[]

    for (left_center, right_center) in zip(left_path, right_path)
        if any(isnan, Tuple(left_center)) || any(isnan, Tuple(right_center))
            push!(track_width, NaN)
        else
            push!(track_width, abs(Float64(left_center[2]) - Float64(right_center[2])))
        end
    end

    return compression_values, track_width
end

function damper_motion_ratio(steering::Steering,
                                suspension::Suspension;
                                step_size = 1.0)
    compression_values, left_path, _ = wheel_center_path(steering, suspension; step_size = step_size)
    motion_ratio = fill(NaN, length(compression_values))

    damper_travel = [compression / 100.0 * suspension.damper[1].travel for compression in compression_values]

    for index in 2:length(compression_values)
        wheel_travel = abs(Float64(left_path[index][3]) - Float64(left_path[index - 1][3]))
        damper_step = abs(damper_travel[index] - damper_travel[index - 1])

        motion_ratio[index] = wheel_travel <= eps(Float64) ? NaN : damper_step / wheel_travel
    end

    return compression_values, motion_ratio
end

function roll_kinematics(θ::Tuple{T,T,T},
                            chassis::Chassis,
                            steering::Steering,
                            suspension::Suspension;
                            signed = ackermann_ratio_signed(),
                            step_size = 1.0) where {T <: Any}
    roll_values = compression_sweep_values(; step_size = step_size)
    left_camber = fill(NaN, length(roll_values))
    right_camber = fill(NaN, length(roll_values))
    left_wheel_angle = fill(NaN, length(roll_values))
    right_wheel_angle = fill(NaN, length(roll_values))
    track_width = fill(NaN, length(roll_values))
    ackermann_ratio_values = fill(NaN, length(roll_values))

    for (index, roll_state) in enumerate(roll_values)
        steering_copy = clean_error_info_copy(steering)
        suspension_copy = clean_error_info_copy(suspension)

        suspension_copy.damper[1].compression = roll_state
        suspension_copy.damper[2].compression = 100.0 - roll_state

        try
            MMK.update!(θ, steering_copy, suspension_copy)

            left_camber[index] = wheel_camber_angle(suspension_copy, 1)
            right_camber[index] = wheel_camber_angle(suspension_copy, 2)
            left_wheel_angle[index] = steering_copy.δo
            right_wheel_angle[index] = steering_copy.δi

            left_center = wheel_center_vehicle_position(steering_copy, suspension_copy, 1)
            right_center = wheel_center_vehicle_position(steering_copy, suspension_copy, 2)
            track_width[index] = abs(Float64(left_center[2]) - Float64(right_center[2]))

            ackermann_ratio_values[index] = steering_copy.δo == 0.0 ? NaN : ackermannratio(θ, chassis, steering_copy, suspension_copy; signed = signed)
        catch
            left_camber[index] = NaN
            right_camber[index] = NaN
            left_wheel_angle[index] = NaN
            right_wheel_angle[index] = NaN
            track_width[index] = NaN
            ackermann_ratio_values[index] = NaN
        end
    end

    return roll_values, left_camber, right_camber, left_wheel_angle, right_wheel_angle, track_width, ackermann_ratio_values
end
