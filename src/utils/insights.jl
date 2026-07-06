"""
    ackermannratio(angleConfig::Tuple{T,T,T}, 
                        chassis::Chassis, 
                        steering::Steering, 
                        suspension::Suspension) where {T >: Any}


Calculates the Ackermann steering ratio [%] based on the current steering configuration.
A result of 100% indicates a perfect match with ideal Ackermann behavior, while lower values indicate deviation due to geometry, articulation, or linkage constraints.

# Arguments
- `angleConfig::Tuple{T,T,T}`: A tuple `(ϕx, ϕy, ϕz)` representing the current steering angles.
- `chassis::Chassis`: The chassis object containing structural vehicle information such as wheelbase.
- `steering::Steering`: The steering system configuration, used to evaluate current joint and linkage positions.
- `suspension::Suspension`: The suspension model affecting the geometry during steering.

# Description
This function evaluates how closely the current steering geometry approximates ideal Ackermann steering. It works by:
- Computing the `steering_objective`, representing deviation from ideal geometry.
- Retrieving the effective wheelbase from the `Measurements` helper struct.
- Using these values to compute the ratio:
        AckermannRatio = (wheel_base / (wheel_base + objective)) * 100
- A result of 100% indicates a perfect match with ideal Ackermann behavior, while lower values indicate deviation due to geometry, articulation, or linkage constraints.

# Returns 
- Float64: The computed Ackermann ratio in percent [%].
"""
function ackermannratio(angleConfig::Tuple{T,T,T},chassis::Chassis, steering::Steering, suspension::Suspension; signed = false) where {T >: Any}
    #wheel_offset                    # Distance Rotationspoint and Wheelcenter
    #offset = wheel_offset * sind(δo)

    measurment = Measurements(chassis, steering)
    deviation = ackermann_deviation(angleConfig, chassis, steering, suspension)
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
                   ϕ_max::Tuple{T,T,T}; 
                   step_size = 1) where {T <: Any}

Computes a matrix of turning radii over a grid of steering angles (ϕx, ϕz), with ϕy held constant.

Arguments:
- `chassis::Chassis`: The vehicle's chassis model, providing wheelbase information.
- `steering::Steering`: The steering system, used to determine steering angles.
- `suspension::Suspension`: The suspension system affecting the steering geometry.
- `ϕ_max::Tuple{T,T,T}`: Maximum angle values (ϕx_max, ϕy_fixed, ϕz_max) defining the steering input space.

# Keywords
- `step_size`: Step size for the ϕx and ϕz sweep. Defaults to 1 degree.

# Description:
This function creates a 2D grid over the (ϕx, ϕz) angle space and computes the turning radius for each combination.
For each angle configuration:
- The steering system is updated via `update!`.
- The outer wheel angle δo is used to compute the radius as:

      radius = wheel_base / sin(δo)

- If δo is 0.0, the radius is set to NaN.

# Returns:
- A 2D Array of Float64 values representing turning radii in millimeters across the ϕx–ϕz angle grid.
"""
function steering_radii(chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension, 
                            ϕ_max::Tuple{T,T,T}; 
                            step_size = 1 ) where {T <: Any}

    ϕx_max , ϕy, ϕz_max = ϕ_max
    ϕ_matrix = [(ϕx, ϕy, ϕz) for ϕx in 0:step_size:ϕx_max, ϕz in 0:step_size:ϕz_max]
    radii = [ 0.0 for x in 0:step_size:ϕx_max, z in 0:step_size:ϕz_max]


    for ϕ in ϕ_matrix
        ϕx, ϕy, ϕz = ϕ
        update!(ϕ, steering, suspension)

        if steering.δo == 0.0
            radii[ϕx+1,ϕz+1] = NaN
        else
    
        measurment = Measurements(chassis, steering)
        δo = deg2rad(steering.δo)
    
        radii[ϕx + 1,ϕz + 1] = measurment.wheel_base / sin(δo)
        end
    end 

    return radii

end

"""
    steering_radii_ϕz(ϕx::T, ϕy::T, ϕz_max::T, 
                      chassis::Chassis, 
                      steering::Steering, 
                      suspension::Suspension; 
                      step_size = 1) where {T <: Any}

Computes a list of turning radii for a sweep of ϕz (inner wheel angle), with fixed ϕx and ϕy.

# Arguments:
- `ϕx::T`: Steering angle around the x-axis (fixed).
- `ϕy::T`: Steering angle around the y-axis (fixed).
- `ϕz_max::T`: Maximum steering angle to sweep over ϕz (in degrees).
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system, providing access to current δo (outer steering angle).
- `suspension::Suspension`: The suspension model used in geometry update.

# Keywords
- `step_size`: Angle increment for ϕz sweep. Defaults to 1 degree.

Description:
This function iterates over the range `0:step_size:ϕz_max`, updating the steering and suspension system
for each ϕz value (with ϕx and ϕy held constant). It computes the turning radius using:

    radius = wheel_base / sin(δo)

If `δo == 0.0`, the function returns `NaN` for that configuration.

# Returns:
- A 1D Array of Float64 values representing turning radii [mm] across the ϕz angle sweep.
"""
function steering_radii_ϕz(ϕx::T,
                            ϕy::T,
                            ϕz_max::T, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension; 
                            signed = false,
                            step_size = 1 ) where {T <: Any}

    ϕ_matrix = [i for i in 0:step_size:ϕz_max]
    radii = []


    for ϕ in ϕ_matrix
        ϕz = ϕ
        update!((ϕx, ϕy, ϕz), steering, suspension)

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
    ackermannratio_ϕz(ϕx::T, ϕy::T, ϕz_max::T,
                      chassis::Chassis,
                      steering::Steering,
                      suspension::Suspension;
                      step_size = 1) where {T <: Any}

Computes the Ackermann ratio [%] across a sweep of ϕz values, with fixed ϕx and ϕy angles.

# Arguments:
- `ϕx::T`: Fixed steering angle around the x-axis.
- `ϕy::T`: Fixed steering angle around the y-axis.
- `ϕz_max::T`: Maximum value for ϕz (inner wheel angle in degrees).
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system used to retrieve joint states and δo.
- `suspension::Suspension`: The suspension system affecting wheel geometry.

# Keywords
- `step_size`: Step size for the ϕz sweep (in degrees). Defaults to 1.

# Description:
This function sweeps ϕz from 0 to `ϕz_max` in the specified step size and computes the Ackermann ratio
at each step using the current steering and suspension configuration.

If the outer wheel angle `δo` is 0.0 (undefined steering geometry), the function substitutes `ϕz + 1` to avoid division by zero.

# Returns:
- A 1D Array of Float64 values representing the Ackermann ratio [%] across the ϕz sweep.
"""
function ackermannratio_ϕz(ϕx::T, 
                            ϕy::T, 
                            ϕz_max::T, 
                            chassis::Chassis, 
                            steering::Steering, 
                            suspension::Suspension; 
                            step_size = 1 ) where {T <: Any}

    ϕ_matrix = [i for i in 0:step_size:ϕz_max]
    ratio = []


    for ϕ in ϕ_matrix
        ϕz = ϕ
        update!((ϕx, ϕy, ϕz), steering, suspension)

        if steering.δo == 0.0
            push!(ratio,ackermannratio((ϕx, ϕy, ϕz+1),chassis, steering, suspension; signed = signed))
        else
            push!(ratio,ackermannratio((ϕx, ϕy, ϕz),chassis, steering, suspension; signed = signed))
        end
    end 
    return ratio

end

"""
    ackermannratio_surface(chassis::Chassis, 
                           steering::Steering, 
                           suspension::Suspension, 
                           ϕ_max::Tuple{T,T,T}; 
                           step_size = 1) where {T <: Any}

Computes a 2D surface of Ackermann ratio [%] values over a grid of (ϕx, ϕz) steering angles, with ϕy fixed.

# Arguments:
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The steering system model containing joint and linkage data.
- `suspension::Suspension`: The suspension system that affects wheel positioning.
- `ϕ_max::Tuple{T,T,T}`: Tuple `(ϕx_max, ϕy_fixed, ϕz_max)` specifying angle sweep limits.

# Keywords
- `step_size`: Increment size for the ϕx and ϕz sweep. Default is 1 degree.

# Description:
This function creates a 2D grid of angle configurations for ϕx and ϕz (holding ϕy constant) and computes
the Ackermann ratio at each grid point using the current steering and suspension configuration.

If the outer wheel steering angle `δo` is zero, the corresponding ratio value is set to NaN to avoid invalid computation.

# Returns:
- A 2D Array of Float64 values representing the Ackermann ratio [%] across the ϕx–ϕz parameter space.
"""
function ackermannratio_surface(chassis::Chassis, 
                                    steering::Steering, 
                                    suspension::Suspension, 
                                    ϕ_max::Tuple{T,T,T};
                                    signed = false,
                                    step_size = 1 ) where {T <: Any}

    ϕx_max , ϕy, ϕz_max = ϕ_max
    ϕ_matrix = [(ϕx, ϕy, ϕz) for ϕx in 0:step_size:ϕx_max, ϕz in 0:step_size:ϕz_max]
    ratio = [ 0.0 for x in 0:step_size:ϕx_max, z in 0:step_size:ϕz_max]

    for ϕ in ϕ_matrix
        ϕx, ϕy, ϕz = ϕ
        update!(ϕ, steering, suspension)

        if steering.δo == 0.0
            ratio[ϕx+1,ϕz+1] = NaN
        else
    
        ratio[ϕx+1,ϕz+1] = ackermannratio(ϕ,chassis, steering, suspension; signed = signed)
        end
    end 
    return ratio
end
