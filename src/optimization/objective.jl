"""
    steering_objective(angleConfig::Tuple{T,T}, measurements::Measurements, steering::Steering, suspension::Suspension)

    Calculates the distance between the optimum point of intersection of the wheel axis (normally on the rear wheel axis) and the current point of intersection of the axis.

# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
    - `θx`: Angle of rotation of the rotation component around the x-axis
    - `θz`: Angle of rotation of the rotation component around the z-axis
- `measurements::Measurements`: Instance of a specific all relevant Measurements of the vehicle
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns
- Distance between optimal and current intersection point
"""
function steering_objective(angleConfig::Tuple{T,T},chassis::Chassis, steering::Steering, suspension::Suspension) where {T<:Any}
    println("Thread $(Threads.threadid()):> objective")
    # calculate the kinematics in respect of the given angles
    #println("$(steering.rotational_component.x_rotational_radius), $(steering.rotational_component.z_rotational_radius), $(steering.track_lever.length), $(steering.tie_rod.length)\n  ")
    update!(angleConfig, steering, suspension)

    # unpack important measurements
    measurements = Measurements(chassis, steering)
    wheel_base = measurements.wheel_base
    track_width = measurements.track_width



    δi = steering.δi
    δo = steering.δo

    # 1. linear function mx + b
    # i(x): inner angle  | o(x): outer angle
    # 1.1 gradient
    Δxi = wheel_base/tand(δi)
    Δxo = wheel_base/tand(δo)
    Δy = wheel_base

    mi = - (Δy/Δxi)
    mo = - (Δy/Δxo)
 
    # 1.2 shift
    # -> calculation of the intersection with x-axis (only o(x))
    x1 = Δxo - (Δxi + track_width)
    b = -mo * x1


    #1.3 intersection both functions i(x) = o(x)
    #->x-Coordinate
    x2 = (b)/ (mi - mo)
    #->y-Coordinate
    y = mo * x2 + b

    # saves the current best value
    # -> 
    #if typeof(y) == Float64
    #    save_current_best_objective°(steering,abs(y))
    #end

    return abs(y)
end



"""
    objective°(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

The wrapper function of the steering_objective procedure is employed for the purposes of optimisation.
! function°(): symbolises that this function should only be used within the optimisation !


# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
    - `θx`: Angle of rotation of the rotation component around the x-axis
    - `θz`: Angle of rotation of the rotation component around the z-axis
- `measurements::Measurements`: Instance of a specific all relevant Measurements of the vehicle
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns
- Distance between optimal and current intersection point

"""
function objective°(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    ################## 
    # Supsystems
    steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    suspension = Suspension(30.0)
    chassis = Chassis()
    # Calculation of objective function 
    return steering_objective((θx,θz),chassis,steering,suspension)

end


