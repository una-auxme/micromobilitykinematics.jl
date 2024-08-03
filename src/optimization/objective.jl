
"""


"""
function steering_objective(angleConfig::Tuple{T,T}, measurments::Measurements, steering::Steering, suspension::Suspension) where {T<:Real}
    
    # unpack important measurments
    θx, θz = angleConfig # angle configuration of the rotary element of the steering
    wheel_base = measurments.wheel_base
    track_width = measurments.track_width

    # calculate the kinematics in respect of the given angles
    update!(angleConfig, steering, suspension)

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