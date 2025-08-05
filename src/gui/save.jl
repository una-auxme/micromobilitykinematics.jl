"""
    geometry_plot(θ::Tuple{T,T,T}, 
                    steering::Steering, 
                    suspension::Suspension) where {T <: Any}

Generates a 3D visualization of the steering geometry for a given steering angle configuration.

# Arguments
- `θ::Tuple{T,T,T}`: A tuple `(θx, θy, θz)` representing the steering angles in radians or degrees, depending on system conventions.
- `steering::Steering`: The steering system object containing joint positions, lever arms, and rotational vectors.
- `suspension::Suspension`: The suspension system object, used during steering kinematics updates.

# Description
This function:
- Updates the steering and suspension configuration using `steeringkinematicsMOVED!`.
- Constructs a 3D plot with key steering components:
  - Rotational axis vectors
  - Left and right steering linkage points
  - Fixed (stationary) suspension elements
- Displays each group of points with `scatter!` and connects them using `lines!`.
- Labels axes and includes the current steering angle values in the plot title.

The result is an intuitive visual representation of the current steering geometry.

# Returns
- `fig::Figure`: A Makie figure containing the 3D steering geometry plot.
"""
function geometry_plot(θ::Tuple{T,T,T}, steering::Steering, suspension::Suspension)where {T <: Any}

    θx, θy, θz = θ 

    steeringkinematicsMOVED!(θ, steering, suspension)

    fig = GLMakie.Figure(size = (900, 600))

    #######################################################| Layout 
    ############| Layout 3D Axis


    ###| Geometry Plot
    geo_ax = GLMakie.Axis3(fig[1:2, 1:3],
                            xlabel = "x in [mm]", 
                            ylabel = "y in [mm]",
                            zlabel = "z in [mm]", 
                            title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)",) #
    geo_ax.aspect = :data




    

    # Limits
    GLMakie.xlims!(geo_ax, -200, 50)
    GLMakie.ylims!(geo_ax, -300, 300)
    GLMakie.zlims!(geo_ax, -200, 50)


    ###| Geometry Plot
    rotational_coponent = [Point3f([0,0,0]),
                            Point3f(steering.vec_x_rotational...),
                            Point3f(steering.vec_z_rotational...)]

    left_steering_connections = [Point3f(steering.vec_z_rotational...),
                                    Point3f(steering.sphere_joints[1]...),
                                    Point3f(steering.circle_joints[1]...),
                                    Point3f(steering.track_lever_mounting_points_ucs[1]...)]

    right_steering_connections = [Point3f(steering.vec_z_rotational...),
                                    Point3f(steering.sphere_joints[2]...),
                                    Point3f(steering.circle_joints[2]...),
                                    Point3f(steering.track_lever_mounting_points_ucs[2]...)]

    # stationary
    stationary = [Point3f(steering.wishbone_ucs_position[1]...),
                    Point3f(steering.wishbone_ucs_position[2]...)]  

    
                    ###| Geometry Plot
    GLMakie.scatter!(geo_ax, rotational_coponent, markersize=10)
    GLMakie.scatter!(geo_ax, left_steering_connections, markersize=10)
    GLMakie.scatter!(geo_ax, right_steering_connections, markersize=10)
    GLMakie.scatter!(geo_ax, stationary, markersize=10)

    GLMakie.lines!(geo_ax, rotational_coponent)
    GLMakie.lines!(geo_ax, left_steering_connections)
    GLMakie.lines!(geo_ax, right_steering_connections)


    return fig
end 



"""
    ackermannratio_θz_plot(θx::T,
                                θy::T,
                                θz_max::T,
                                chassis::Chassis, 
                                steering::Steering, 
                                suspension::Suspension) where{T <: Any}

Creates a 2D plot of the Ackermann ratio across a range of θz values,
for a fixed steering configuration defined by (θx, θy).

# Arguments
- `θx::T`: Fixed steering angle around the x-axis.
- `θy::T`: Fixed steering angle around the y-axis.
- `θz_max::T`: Maximum value of θz to consider for the plot (in degrees).
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The current steering system state.
- `suspension::Suspension`: The suspension model affecting steering geometry.

# Description
This function:
- Computes the Ackermann ratio for values of `θz` ranging from 0 to `θz_max`.
- Plots the ratio over the `θz` interval.
- Adds horizontal dashed lines marking the minimum and maximum ratio values.
- Annotates the extrema with corresponding values for quick interpretation.
- Sets appropriate axis limits and tick marks for a clean, standardized layout.

The plot helps visualize how the Ackermann ratio behaves across a sweep of inner steering angles, supporting design analysis or tuning.

# Returns
- `fig::Figure`: A Makie figure containing the 2D Ackermann ratio plot.
"""
function ackermannratio_θz_plot(args...) where{T <: Any}

    ratio = ackermannratio_θz(args...)

    fig = GLMakie.Figure(size = (900, 600))

    ax_ratio = GLMakie.Axis(fig[1:2, 1:3], 
                            xlabel = "θz in [°]", 
                            ylabel = "ackermannratio [%]", 
                            title = "Ratio for (θx, θy, θz) = (0,0,0)", 
                            xticks = 0:5:40,
                            yticks = 50:5:100)

    # Limits
    GLMakie.xlims!(ax_ratio, 0, 40)
    GLMakie.ylims!(ax_ratio, 50, 105)


    min = minimum(ratio)
    max = maximum(ratio)

    ###| Ratio Plot
    xs = [θz for θz in 0:1:θz_max] 
    GLMakie.lines!(ax_ratio, xs, ratio)

    #
    GLMakie.hlines!(ax_ratio, min, linestyle = :dash, color = :orange)
    GLMakie.hlines!(ax_ratio, max, linestyle = :dash, color = :red)

    text_min = GLMakie.text!(ax_ratio, 
                                Point(30, 55),#Point(xs[end-15], observer_min[]),
                                text = "Min → $(round(min, digits=2))%",
                                align = (:left, :bottom),
                                color = :orange)

    text_max = GLMakie.text!(ax_ratio, 
                                Point(35, 55),#Point(xs[end-15], observer_max[]),
                                text = "Min → $(round(max, digits=2))%",
                                align = (:left, :bottom),
                                color = :red)  

    return fig

end


function radii_plot()




 ###| Radii Plot
    ax_radii = GLMakie.Axis(fig[1:2, 1:3], 
                            xlabel = "θz in [°]", 
                            ylabel = "radius in [mm]", 
                            title = "Radii for (θx, θy, θz) = (0,0,0)",
                            yticks = 0:1000:40000) 
    ax_radii.blockscene.visible[] = false
    # Limits
    GLMakie.xlims!(ax_radii, 0, 35)
    GLMakie.ylims!(ax_radii, 0, 20000)


end




"""
    ackermannratio_sufrace_plot(args...)


    Generates a 3D surface plot of the Ackermann ratio across a grid of steering angles θx and θz.

    # Arguments
    - `args...`: A variadic argument list expected to include:
      - `chassis::Chassis`: Vehicle chassis configuration.
      - `steering::Steering`: Steering system object.
      - `suspension::Suspension`: Suspension model.
      - `(θx_max, θy_fixed, θz_max)::Tuple`: Maximum bounds for θx and θz (sweep dimensions) and a fixed θy value.
    
    # Description
    This function:
    - Computes the Ackermann ratio across a meshgrid of `θx` and `θz` angles while holding `θy` constant.
    - Creates a 3D surface plot of the result using `GLMakie.surface!`.
    - Sets up labeled axes, a fixed colormap (`:darkterrain`), and value ranges for better interpretability.
    - Useful for assessing how different combinations of steering input influence the Ackermann behavior.
    
    Note: The ratio is expected to be precomputed using `ackermannratio_surface`.
    
    # Returns
    - `fig::Figure`: A Makie 3D figure showing the Ackermann ratio surface.
"""
function ackermannratio_sufrace_plot(args...)
    θx_max,θx_max,θz_max = args[4]
    ratio = ackermannratio_surface(args...)

    fig = GLMakie.Figure(size = (900, 1300))

    ratio_su = GLMakie.Axis3(fig[1:2, 1:3],
                            xlabel = "θx in [°]", 
                            ylabel = "θz in [°]",
                            zlabel = "ratio in [%]",
                            zticks = 50:5:100, 
                            title = "Ackermannratio surface",) #
    ratio_su.aspect = :data
    

    # Limits
    GLMakie.xlims!(ratio_su, 0, 15)
    GLMakie.ylims!(ratio_su, 0, 30)
    GLMakie.zlims!(ratio_su, 50, 105)


    GLMakie.surface!(ratio_su, 0:1:θx_max, 0:1:θz_max, ratio; colormap = :darkterrain)


    return fig
end