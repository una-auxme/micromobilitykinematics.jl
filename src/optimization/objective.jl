"""
    steering_objective(angleConfig::Tuple{T,T}, measurments::Measurements, steering::Steering, suspension::Suspension)

    Calculates the distance between the optimum point of intersection of the wheel axis (normally on the rear wheel axis) and the current point of intersection of the axis.

    #Arguments
    -`angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component ist rotated
        -`θx`: Angle of rotation of the rotation component around the x-axis
        -`θz`: Angle of rotation of the rotation component around the z-axis
    -`measurments::Measurements`: Instance of a specific all relevant Measurements of the vehicle
    -`steering::Steering`: Instance of a specific steering
    -`suspension::Suspension`: Instance of a specific suspension

    #Returns
    - Distance between optimal and current intersection point
"""
function steering_objective(angleConfig::Tuple{T,T},chassi::Chassi, steering::Steering, suspension::Suspension) where {T<:Union{<:Real, VariableRef} }
    # calculate the kinematics in respect of the given angles
    update!(angleConfig, steering, suspension)

    # unpack important measurments
    measurements = Measurements(chassi, steering)
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

    return abs(y)
end


function objective(θx,θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    ################## 
    # Supsystems
    steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    suspension = Suspension(30.0)
    chassi = Chassi()
    @show typeof(θx)
    # Calculation of objective function 
    return steering_objective((θx,θz),chassi,steering,suspension)

end