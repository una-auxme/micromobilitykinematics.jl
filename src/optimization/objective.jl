"""
    ackermann_deviation(θ::Tuple{T,T,T}, chassis::Chassis, steering::Steering, suspension::Suspension) where {T<:Any}

    Calculates the distance between the optimum point of intersection of the wheel axis (normally on the rear wheel axis) and the current point of intersection of the axis.

# Arguments
- `θ::Tuple{T,T,T}`: angles (θx,θy,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `measurements::Measurements`: Instance of a specific all relevant Measurements of the vehicle
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns
- Distance between optimal and current intersection point
"""
function ackermann_deviation(θ::Tuple{T,T,T}, 
                                chassis::Chassis, 
                                steering::Steering, 
                                suspension::Suspension) where {T<:Any}

    # --- Update steering and suspension system with new steering angles ---
    update!(θ, steering, suspension)

    # --- Unpack important measurements from the vehicle model ---
    measurements = Measurements(chassis, steering)
    wheel_base = measurements.wheel_base
    track_width = measurements.track_width


    # --- Get inner and outer steering angles ---
    δi = steering.δi
    δo = steering.δo

    # --- Define linear functions mx + b for steering geometry ---
    # --- i(x): inner angle  | o(x): outer angle
    # 1.1 Compute gradients (m) of the linear functions for inner and outer wheels
    Δxi = wheel_base/tand(δi)
    Δxo = wheel_base/tand(δo)
    Δy = wheel_base

    mi = - (Δy/Δxi)
    mo = - (Δy/Δxo)
 
    # 1.2 shift
    # -> calculation of the intersection with x-axis (only o(x))
    x1 = Δxo - (Δxi + track_width)
    b = -mo * x1


    # 1.3 intersection both functions i(x) = o(x)
    # -> x-Coordinate
    x2 = (b)/ (mi - mo)
    # -> y-Coordinate
    y = mo * x2 + b


    return abs(y)
end



"""
    ackermann_deviation_at_pose(θx, θy, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

    Calculates the Ackerman deviation for a given steering angle position.

# Arguments
- `θx`: Angle of rotation of the rotation component around the x-axis
- `θy`: Angle of rotation of the rotation component around the y-axis
- `θz`: Angle of rotation of the rotation component around the z-axis
- `x_rotational_radius`: length of the rotation component around the x-axis
- `z_rotational_radius`: length of the rotation component around the z-axis
- `track_lever_length`: length of the track lever
- `tie_rod_length`: length of the tie rod

# Returns
- Distance between optimal and current intersection point (known as ackermann deviation)

"""
function ackermann_deviation_for_pose(θx, 
                    θy,     
                    θz, 
                    x_rotational_radius, 
                    z_rotational_radius, 
                    track_lever_length, 
                    tie_rod_length)

        try 
            # --- Init subsystems ---
            steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
            suspension = Suspension((30,30))
            chassis = Chassis()

             # --- Angle preprocessing ---
            θx__ = θx*100
            θx_ = Int(round(θx__))
            θx = θx_ / 100

            θy__ = θy*100
            θy_ = Int(round(θy__))
            θy = θy_ / 100

            θz__ = θz*100
            θz_ = Int(round(θz__))
            θz = θz_ / 100

            # --- Evaluate Ackermann deviation for given pose --- 
            cost = try
                ackermann_deviation((θx,θy,θz), chassis, steering, suspension)
            catch err
                # --- Invalid point → assign high cost ---
                @warn "Error by steering_objective($θx, $θy, $θz): $err"
                return Inf
            end

            return cost

    catch err
        # --- Invalid point → assign high cost ---
        @warn "Error by steering_objective($θx, $θy, $θz): $err"
        return Inf
    end


end


"""
    ackermann_deviation_over_range(θx_max, θy_max, θz_max, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

    Calculates the Ackerman deviation for a given steering angle position.
    !!! θy_max describes the desired rotation around the y-axis. The optimization is performed using the search space θx_max, θz_max with a constant θy_max.!!!
    

# Arguments
- `θx_max`: The maximal angle of rotation of the rotation component around the x-axis
- `θy_max`: !!! Angle of rotation of the rotation component around the y-axis !!!
- `θz_max`: The maximal angle of rotation of the rotation component around the z-axis
- `x_rotational_radius`: length of the rotation component around the x-axis
- `z_rotational_radius`: length of the rotation component around the z-axis
- `track_lever_length`: length of the track lever
- `tie_rod_length`: length of the tie rod

# Returns
- Distance between optimal and current intersection point (known as ackermann deviation)

"""
function ackermann_deviation_over_range(θx_max, 
                                    θy_max, 
                                    θz_max, 
                                    x_rotational_radius, 
                                    z_rotational_radius, 
                                    track_lever_length, 
                                    tie_rod_length)
    try
        # --- Init subsystems ---
        steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
        suspension = Suspension((30,30))
        chassis = Chassis()

        # --- Angle preprocessing ---
        θy_max = θy_max*100
        θy_ = Int(round(θy_max))
        θy = θy_ / 100

        
        θx_max = Int(round(θx_max))
        θz_max = Int(round(θz_max))

        step_size = 1.0
        θ_tuple = [(i, j) for i in 0.0:step_size:θx_max, j in 0.0:step_size:θz_max]


        # --- Main cost loop ---
        cost = 0
        for i in 1:Int((θx_max/step_size)+1)
            for θ in θ_tuple[i,:] 
                if θ != (0,0)

                    # Compose full angle vector
                    θx,θz = θ
                    θ_ = (θx,θy,θz)

                    # Weighting: penalize large angles
                    # weight = 1.0 - ((θx / θx_max)^2 + (θz / θz_max)^2) / 2
                    α = 1.5           # 1-5    – je höher, desto schneller fällt Gewicht ab.
                    weight = exp(-α * ((θx/θx_max)^2 + (θz/θz_max)^2))

                    # Evaluate error
                    error = try
                        ackermann_deviation(θ_, chassis, steering, suspension)
                    catch err
                        @warn "Error in ackermann_deviation(θ=$θ): $err"
                        return Inf  # ungültiger Punkt → hohe Kosten
                    end
                  
                    cost += error^2 * weight
                end   
            end
        end 

        return cost
    catch err
        @warn "Error in steering_geometry_cost°: $err"
        return Inf
    end
   
end

