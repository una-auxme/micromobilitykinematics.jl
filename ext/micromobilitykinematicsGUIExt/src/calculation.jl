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
                            suspension::Suspension) where {T >: Any}
    #wheel_offset                    # Distance Rotationspoint and Wheelcenter
    #offset = wheel_offset * sind(δo)

    measurment = Measurements(chassis, steering)
    objective = abs(ackermann_deviation(θ, chassis, steering, suspension))

    L = objective + measurment.wheel_base #+ offset

    return (measurment.wheel_base/L)*100

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
                            step_size = 1 ) where {T <: Any}

    θ_matrix = [i for i in 0:step_size:θz_max]
    ratio = []


    for θ in θ_matrix
        θz = θ
        MMK.update!((θx, θy, θz), steering, suspension)



        if steering.δo == 0.0
            push!(ratio,ackermannratio((θx, θy, θz+1.0),chassis, steering, suspension))
        else
            push!(ratio,ackermannratio((θx, θy, θz),chassis, steering, suspension))
        end
    end 
    return ratio

end


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
function ackermannratio_surface(ackermannratio_surface::Chassis, 
                                    steering::Steering, 
                                    suspension::Suspension, 
                                    θ_max::Tuple{T,T,T};
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
    
        ratio[θx_i+1,θz_i+1] = ackermannratio(θ,chassis, steering, suspension)
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
        MMK.update!(θ, steering, suspension)

        θx_i = Int(round(θx))
        θz_i = Int(round(θz))

        if steering.δo == 0.0
            deviation[θx_i+1,θz_i+1] = NaN
        else
    
        deviation[θx_i+1,θz_i+1] = ackermann_deviation(θ,chassis, steering, suspension)
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
        update!(θ, steering, suspension)

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
        update!(θ, steering, suspension)

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


    compr_matrix = [(l, r) for l in 1.0:step_size:100, r in 1.0:step_size:100]
    δi = [ 0.0 for l in 1:step_size:100, r in 1:step_size:100]
    δo = [ 0.0 for l in 1:step_size:100, r in 1:step_size:100]

    for compr in compr_matrix
        l_compr, r_compr = compr


        suspension.damper[1].compression = l_compr
        suspension.damper[2].compression = r_compr
        l_compr_i = Int(round(l_compr))
        r_compr_i = Int(round(r_compr))

        try
            update!(θ, steering, suspension)

            δi[l_compr_i,r_compr_i] = steering.δi
            δo[l_compr_i,r_compr_i] = steering.δo
        catch err
            #println(l_compr, r_compr)
            δi[l_compr_i,r_compr_i] = NaN
            δo[l_compr_i,r_compr_i] = NaN
        end
        
       
    end 
    
    return δi, δo
end