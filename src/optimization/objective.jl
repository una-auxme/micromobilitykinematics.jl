"""
    ackermann_deviation(ϕ::Tuple{T,T,T}, chassis::Chassis, steering::Steering, suspension::Suspension) where {T<:Any}

    Calculates the distance between the optimum point of intersection of the wheel axis (normally on the rear wheel axis) and the current point of intersection of the axis.

# Arguments
- `ϕ::Tuple{T,T,T}`: angles (ϕx,ϕy,ϕz) in which the rotational component is rotated
        - `ϕx`: Angle of rotation of the rotation component around the x-axis
        - `ϕy`: Angle of rotation of the rotation component around the y-axis
        - `ϕz`: Angle of rotation of the rotation component around the z-axis
- `measurements::Measurements`: Instance of a specific all relevant Measurements of the vehicle
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns
- Distance between optimal and current intersection point
"""
function ackermann_deviation(ϕ::Tuple{T,T,T}, 
                                chassis::Chassis, 
                                steering::Steering, 
                                suspension::Suspension) where {T<:Any}

    # --- Update steering and suspension system with new steering angles ---
    update!(ϕ, steering, suspension)

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

    return y
end


"""
    ackermann_deviation_at_pose(ϕx, ϕy, ϕz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

    Calculates the Ackerman deviation for a given steering angle position.

# Arguments
- `ϕx`: Angle of rotation of the rotation component around the x-axis
- `ϕy`: Angle of rotation of the rotation component around the y-axis
- `ϕz`: Angle of rotation of the rotation component around the z-axis
- `x_rotational_radius`: length of the rotation component around the x-axis
- `z_rotational_radius`: length of the rotation component around the z-axis
- `track_lever_length`: length of the track lever
- `tie_rod_length`: length of the tie rod

# Returns
- Distance between optimal and current intersection point (known as ackermann deviation)

"""
function ackermann_deviation_for_pose(ϕx, 
                    ϕy,     
                    ϕz, 
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
            ϕx__ = ϕx*100
            ϕx_ = Int(round(ϕx__))
            ϕx = ϕx_ / 100

            ϕy__ = ϕy*100
            ϕy_ = Int(round(ϕy__))
            ϕy = ϕy_ / 100

            ϕz__ = ϕz*100
            ϕz_ = Int(round(ϕz__))
            ϕz = ϕz_ / 100

            # --- Evaluate Ackermann deviation for given pose --- 
            cost = try
                abs(ackermann_deviation((ϕx,ϕy,ϕz), chassis, steering, suspension))
            catch err
                # --- Invalid point → assign high cost ---
                @warn "Error by ackermann_deviation($ϕx, $ϕy, $ϕz): $err"
                return Inf
            end

            return cost

    catch err
        # --- Invalid point → assign high cost ---
        @warn "Error by ackermann_deviation($ϕx, $ϕy, $ϕz): $err"
        return Inf
    end


end


"""
    ackermann_deviation_over_range(ϕx_max, ϕy_max, ϕz_max, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

    Calculates the Ackerman deviation for a given steering angle position.
    !!! ϕy_max describes the desired rotation around the y-axis. The optimization is performed using the search space ϕx_max, ϕz_max with a constant ϕy_max.!!!
    

# Arguments
- `ϕx_max`: The maximal angle of rotation of the rotation component around the x-axis
- `ϕy_max`: !!! Angle of rotation of the rotation component around the y-axis !!!
- `ϕz_max`: The maximal angle of rotation of the rotation component around the z-axis
- `x_rotational_radius`: length of the rotation component around the x-axis
- `z_rotational_radius`: length of the rotation component around the z-axis
- `track_lever_length`: length of the track lever
- `tie_rod_length`: length of the tie rod

# Returns
- Distance between optimal and current intersection point (known as ackermann deviation)

"""
function ackermann_deviation_over_range(ϕx_max, 
                                    ϕy_max, 
                                    ϕz_max, 
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
        ϕy_max = ϕy_max*100
        ϕy_ = Int(round(ϕy_max))
        ϕy = ϕy_ / 100

        
        ϕx_max = Int(round(ϕx_max))
        ϕz_max = Int(round(ϕz_max))

        step_size = 1.0
        ϕ_tuple = [(i, j) for i in 0.0:step_size:ϕx_max, j in 0.0:step_size:ϕz_max]


        # --- Main cost loop ---
        cost = 0
        for i in 1:Int((ϕx_max/step_size)+1)
            for ϕ in ϕ_tuple[i,:] 
                if ϕ != (0,0)

                    # Compose full angle vector
                    ϕx,ϕz = ϕ
                    ϕ_ = (ϕx,ϕy,ϕz)

                    # Weighting: penalize large angles
                    # weight = 1.0 - ((ϕx / ϕx_max)^2 + (ϕz / ϕz_max)^2) / 2
                    α = 1.5           # 1-5    – je höher, desto schneller fällt Gewicht ab.
                    weight = exp(-α * ((ϕx/ϕx_max)^2 + (ϕz/ϕz_max)^2))

                    # Evaluate error
                    error = try
                        abs(ackermann_deviation(ϕ_, chassis, steering, suspension))
                    catch err
                        @warn "Error in ackermann_deviation(ϕ=$ϕ): $err"
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

