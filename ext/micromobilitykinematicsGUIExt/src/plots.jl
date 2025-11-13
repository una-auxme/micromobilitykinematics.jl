"""
    geom_plot!(fig,section_plot, steering)

Initializes and renders the 3D geometry plot of the steering system into the specified figure.

# Arguments
- `fig`: A `GLMakie.Figure` where the 3D plot will be drawn.
- `section_plot`: A `PlotSection` object that holds references to the axes and observables for plotting.
- `steering`: The `Steering` object containing all relevant geometric data (e.g., joint positions and vectors).

# Description
This function sets up a 3D axis at the `section_plot` slot position and visualizes the steering geometry. It performs the following steps:
- Initializes an `Axis3` with appropriate labels, title, aspect ratio, and axis limits.
- Extracts relevant geometry data from the `steering` object, including:
  - Rotational vectors
  - Left and right linkage components
  - Stationary suspension positions
- Wraps all geometry elements in `Observable`s for dynamic updates.
- Plots the components using `scatter!` and connects key points using `lines!`.

These `Observable`s can later be updated reactively as the user modifies input parameters, providing a real-time visualization of the system.

# Returns
Nothing. Updates `section_plot` in-place by attaching the plot and setting its observables.
"""
function geom_plot!(fig, section_plot, steering, suspension)

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    ############| Geometry Scene
    section_plot.ax_geom = GLMakie.Axis3(fig[row, col],
                                            xlabel = "x in [mm]", 
                                            ylabel = "y in [mm]",
                                            zlabel = "z in [mm]", 
                                            title = "Steering geometry for (θx, θy, θz) = (0,0,0)",) #
    section_plot.ax_geom.aspect = :data

    # Limits
    GLMakie.xlims!(section_plot.ax_geom, -200, 50)
    GLMakie.ylims!(section_plot.ax_geom, -300, 300)
    GLMakie.zlims!(section_plot.ax_geom, -200, 50)


    ############| Geometry Data
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
                        
    # wishbone

    conversion = (i,pos) -> steering.wishbone_ucs_position[i] + pos

    left_lower_wishbone_axis = [Point3f(steering.wishbone_ucs_position[1]...),
                                Point3f(conversion(1, suspension.lowerwishbone[1].bearing_front)...)]
    right_lower_wishbone_axis = [Point3f(steering.wishbone_ucs_position[2]...),
                                Point3f(conversion(2, suspension.lowerwishbone[2].bearing_front)...)]

    left_upper_wishbone_axis = [Point3f(conversion(1, suspension.upperwishbone[1].bearing_rear)...),
                                Point3f(conversion(1, suspension.upperwishbone[1].bearing_front)...)]
    right_upper_wishbone_axis = [Point3f(conversion(2, suspension.upperwishbone[2].bearing_rear)...),
                                Point3f(conversion(2, suspension.upperwishbone[2].bearing_front)...)]              

    left_wishbone_sphere_joint = [Point3f(conversion(1, suspension.lowerwishbone[1].sphere_joint)...),
                                   Point3f(conversion(1, suspension.upperwishbone[1].sphere_joint)...)]
    right_wishbone_sphere_joint = [Point3f(conversion(2, suspension.lowerwishbone[2].sphere_joint .*[1.0, -1.0, 1.0])...),
                                   Point3f(conversion(2, suspension.upperwishbone[2].sphere_joint .*[1.0, -1.0, 1.0])...)]


    left_lower_wishbone = [Point3f(conversion(1, suspension.lowerwishbone[1].bearing_rear + [suspension.lowerwishbone[1].distance_to_joint_x, 0, 0])...),
                            Point3f(conversion(1, suspension.lowerwishbone[1].sphere_joint)...)]   

    left_upper_wishbone = [Point3f(conversion(1, suspension.upperwishbone[1].bearing_rear + [suspension.upperwishbone[1].distance_to_joint_x, 0, 0] )...),
                            Point3f(conversion(1, suspension.upperwishbone[1].sphere_joint)...)]                     

    right_lower_wishbone = [Point3f(conversion(2, (suspension.lowerwishbone[2].bearing_rear + [suspension.lowerwishbone[2].distance_to_joint_x, 0, 0])  )...),
                            Point3f(conversion(2, suspension.lowerwishbone[2].sphere_joint.*[1.0, -1.0, 1.0])...)] 

    right_upper_wishbone = [Point3f(conversion(2, suspension.upperwishbone[2].bearing_rear + [suspension.upperwishbone[2].distance_to_joint_x, 0, 0] )...),
                            Point3f(conversion(2, suspension.upperwishbone[2].sphere_joint.*[1.0, -1.0, 1.0])...)] 

    # Damper

    left_damper = [Point3f(conversion(1,  suspension.damper[1].upper_fixture)...),
                         Point3f(conversion(1, suspension.damper[1].lower_fixture)...)]

    right_damper = [Point3f(conversion(2, suspension.damper[2].upper_fixture .*[1.0, -1.0, 1.0])...),
                         Point3f(conversion(2, suspension.damper[2].lower_fixture .*[1.0, -1.0, 1.0])...)]


    ############| Geometry Observervar                
    section_plot.obs_rotation = Observable(rotational_coponent)
    section_plot.obs_geom_left = Observable(left_steering_connections)
    section_plot.obs_geom_right = Observable(right_steering_connections)
    section_plot.obs_stationary = Observable(stationary)
    ############| Suspension Observervar  
    section_plot.obs_left_lower_wishbone_axis = Observable(left_lower_wishbone_axis)
    section_plot.obs_right_lower_wishbone_axis = Observable(right_lower_wishbone_axis)

    section_plot.obs_left_upper_wishbone_axis = Observable(left_upper_wishbone_axis)
    section_plot.obs_right_upper_wishbone_axis = Observable(right_upper_wishbone_axis)

    section_plot.obs_left_wishbone_sphere_joint = Observable(left_wishbone_sphere_joint)
    section_plot.obs_right_wishbone_sphere_joint = Observable(right_wishbone_sphere_joint)

    section_plot.obs_left_lower_wishbone = Observable(left_lower_wishbone)
    section_plot.obs_left_upper_wishbone = Observable(left_upper_wishbone)
    section_plot.obs_right_lower_wishbone = Observable(right_lower_wishbone)
    section_plot.obs_right_upper_wishbone = Observable(right_upper_wishbone)

    section_plot.obs_left_damper = Observable(left_damper)
    section_plot.obs_right_damper = Observable(right_damper)



    ############| Geometry Ploting   
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_rotation, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_geom_left, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_geom_right, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_stationary, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_lower_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_lower_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_upper_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_upper_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_wishbone_sphere_joint, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_wishbone_sphere_joint, markersize=10; color = :black)

    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_lower_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_upper_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_lower_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_upper_wishbone, markersize=10; color = :black)

    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_left_damper, markersize=10; color = :black)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_right_damper, markersize=10; color = :black)


    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_rotation)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_geom_left)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_geom_right)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_lower_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_lower_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_upper_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_upper_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_wishbone_sphere_joint; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_wishbone_sphere_joint; linestyle = :dash, color = :black)

    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_lower_wishbone; color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_upper_wishbone; color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_lower_wishbone; color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_upper_wishbone; color = :black)

    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_left_damper; linestyle = :dash, color = :black)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_right_damper; linestyle = :dash, color = :black)

end 


"""
    radii_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


    Initializes and renders a 2D plot of the turning radius versus θz (steering angle around z-axis).

    # Arguments
    - `fig`: A `GLMakie.Figure` where the plot will be drawn.
    - `section_plot`: A `PlotSection` object that holds plot axes and observables.
    - `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining the steering angle limits.
    - `chassis`: The `Chassis` object representing the vehicle structure.
    - `steering`: The `Steering` object containing steering-related geometry and states.
    - `suspension`: The `Suspension` system affecting wheel positioning and kinematics.
    
    # Description
    This function:
    - Sets up a 2D `Axis` for plotting radius values over a sweep of θz (0 to θz_max).
    - Initially computes the turning radius using `steering_radii_θz(...)` with θx = 1, θy = 0.
    - Stores the results in an `Observable`, allowing reactive updates later.
    - Draws the radius curve over the θz range using `lines!`.
    - Sets custom axis limits and hides the plot by default (`blockscene.visible[] = false`).
    
    This plot helps evaluate the turning behavior of the vehicle across varying steering input on the z-axis.
    
    # Returns
    Nothing. Updates the `section_plot` in-place with a new axis and observable data
"""
function radii_plot!(fig,section_plot, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Radii θz Scene
    section_plot.ax_radii = GLMakie.Axis(fig[row, col], 
                            xlabel = "θz in [°]", 
                            ylabel = "radius in [mm]", 
                            title = "Radii for (θx, θy, θz) = (0,0,0)",
                            yticks = 0:5000:40000) 
    section_plot.ax_radii.blockscene.visible[] = false
    # Limits
    GLMakie.xlims!(section_plot.ax_radii, 0, 35)
    GLMakie.ylims!(section_plot.ax_radii, 0, 20000)

    #GLMakie.autolimits!(ax_radii)

    ############| Radii θz Data
    radii_θz = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)

    ############| Radii θz Observervar 
    section_plot.obs_radii_θz = Observable(radii_θz)


    ############| Radii θz Ploting  
    xs = [θz for θz in 0.0:1.0:θz_max] 

    GLMakie.lines!(section_plot.ax_radii, xs, section_plot.obs_radii_θz)

end


"""
    ratio_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


Generates a 2D plot of the Ackermann ratio across a range of θz values and highlights the minimum and maximum ratio.

# Arguments
- `fig`: A `GLMakie.Figure` in which the plot will be rendered.
- `section_plot`: A `PlotSection` object containing axes and observables for plotting.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining the range limits of the steering angles.
- `chassis`: The `Chassis` system of the vehicle.
- `steering`: The `Steering` configuration containing kinematic data.
- `suspension`: The `Suspension` model influencing geometry.

# Description
This function:
- Initializes a 2D axis with labeled ticks and axis limits.
- Computes the Ackermann ratio for θz from `0` to `θz_max` at fixed `θx=0`, `θy=0`.
- Stores the data in observables for reactive updates.
- Plots the Ackermann ratio curve and marks the min/max values with:
  - Dashed horizontal lines
  - Dynamic text labels updated via `lift(...)`
- Hides the plot initially (`blockscene.visible[] = false`) until it is selected via the UI.

This visualization is useful for analyzing how closely the steering geometry adheres to ideal Ackermann behavior as inner steering angle changes.

# Returns
Nothing. Updates the `section_plot` object in-place with the axis, plotted data, and labeled extremes.
"""
function ratio_plot!(fig,section_plot, θ_max, chassis, steering, suspension)

    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]


    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_ratio = GLMakie.Axis(fig[row, col], 
                                            xlabel = "θz in [°]", 
                                            ylabel = "ackermannratio [%]", 
                                            title = "Ratio for (θx, θy, θz) = (0,0,0)", 
                                            xticks = 0:5:40,
                                            yticks = 30:5:100)

    section_plot.ax_ratio.blockscene.visible[] = false


    # Limits
    GLMakie.xlims!(section_plot.ax_ratio, 0, 40)
    GLMakie.ylims!(section_plot.ax_ratio, 30, 105)

    ############| Ackermannratio Data

    ratio = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)

    ############| Ackermannratio Observervar 

    section_plot.obs_ratio_θz = Observable(ratio)
    section_plot.obs_ratio_min = Observable(minimum(ratio))
    section_plot.obs_ratio_max = Observable(maximum(ratio))



    ############| Ackermannratio Ploting  
    xs = [θz for θz in 0.0:1.0:θz_max] 

    GLMakie.lines!(section_plot.ax_ratio, xs, section_plot.obs_ratio_θz)

    #
    GLMakie.hlines!(section_plot.ax_ratio, section_plot.obs_ratio_min, linestyle = :dash, color = :orange)
    GLMakie.hlines!(section_plot.ax_ratio, section_plot.obs_ratio_max, linestyle = :dash, color = :red)


    section_plot.txt_ratio_min = GLMakie.text!(section_plot.ax_ratio, 
                                                Point(30, 55),#Point(xs[end-15], observer_min[]),
                                                text = "Min → $(round(section_plot.obs_ratio_min[], digits=2))%",
                                                align = (:left, :bottom),
                                                color = :orange)
    
    section_plot.txt_ratio_max = GLMakie.text!(section_plot.ax_ratio, 
                                                Point(35, 55),#Point(xs[end-15], observer_max[]),
                                                text = "Max → $(round(section_plot.obs_ratio_max[], digits=2))%",
                                                align = (:left, :bottom),
                                                color = :red)  

    lift(section_plot.obs_ratio_min ) do val
        section_plot.txt_ratio_min.text = "Min → $(round(val, digits=2))%"
    end

    lift(section_plot.obs_ratio_max) do val
        section_plot.txt_ratio_max.text = "Max → $(round(val, digits=2))%"
    end

end



"""
    ratio_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)

Creates a 3D surface plot of the Ackermann ratio over θx and θz steering angles.

# Arguments
- `fig`: A `GLMakie.Figure` where the 3D plot will be inserted.
- `section_plot`: A `PlotSection` object that holds the 3D axis and observables.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining the maximum steering angles.
- `chassis`: The `Chassis` model representing the vehicle's structure.
- `steering`: The `Steering` object defining the steering configuration.
- `suspension`: The `Suspension` model affecting steering mechanics.

# Description
This function:
- Initializes a 3D axis (`Axis3`) for displaying the Ackermann ratio surface.
- Computes the Ackermann ratio across a meshgrid of θx and θz values using `ackermannratio_surface(...)`, keeping θy fixed.
- Wraps the result in an `Observable` to allow dynamic updating if needed.
- Draws the surface using `GLMakie.surface!` with a terrain-style colormap.
- Sets axis limits and hides the scene by default (`blockscene.visible[] = false`).

This plot provides an intuitive 3D visualization of how the Ackermann ratio varies across different combinations of steering angles, supporting system tuning and validation.

# Returns
Nothing. Modifies `section_plot` in-place with the 3D plot and reactive data.
"""
function ratio_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_ratio_surface = GLMakie.Axis3(fig[row, col],
                                                    xlabel = "θx in [°]", 
                                                    ylabel = "θz in [°]",
                                                    zlabel = "ratio in [%]",
                                                    zticks = 50:10:100, 
                                                    title = "Ackermannratio surface",) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_ratio_surface.aspect = (1, 1, 1)
    section_plot.ax_ratio_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(section_plot.ax_ratio_surface, 0, 15)
    GLMakie.ylims!(section_plot.ax_ratio_surface, 0, 30)
    GLMakie.zlims!(section_plot.ax_ratio_surface, 50, 105)

    ############| Ackermannratio Data

    ratio_surface = ackermannratio_surface(chassis, steering, suspension, (θx_max,θy,θz_max))


    ############| Ackermannratio Observervar 
    section_plot.obs_ratio_surface = Observable(ratio_surface)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(section_plot.ax_ratio_surface , 0.0:1.0:θx_max, 0.0:1.0:θz_max, section_plot.obs_ratio_surface; colormap = :darkterrain)
    
end



function deviation_plot!(fig,section_plot, θ_max, chassis, steering, suspension)

    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]


    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_deviation = GLMakie.Axis(fig[row, col], 
                                            xlabel = "θz in [°]", 
                                            ylabel = "ackermann_deviation [mm]", 
                                            title = "Ratio for (θx, θy, θz) = (0,0,0)", 
                                            xticks = 0:5:40,
                                            yticks = -500:100:500)

    section_plot.ax_deviation.blockscene.visible[] = false


    # Limits
    GLMakie.xlims!(section_plot.ax_deviation, 0, 40)
    GLMakie.ylims!(section_plot.ax_deviation, -500,500 )

    ############| Ackermannratio Data

    deviation = ackermann_deviation_θz(θx,θy,θz_max,chassis, steering, suspension)

    ############| Ackermannratio Observervar 

    section_plot.obs_deviation_θz = Observable(deviation)
    section_plot.obs_deviation_min = Observable(minimum(deviation))
    section_plot.obs_deviation_max = Observable(maximum(deviation))



    ############| Ackermannratio Ploting  
    xs = [θz for θz in 0.0:1.0:θz_max] 

    GLMakie.lines!(section_plot.ax_deviation, xs, section_plot.obs_deviation_θz)

    #
    GLMakie.hlines!(section_plot.ax_deviation, section_plot.obs_deviation_min, linestyle = :dash, color = :orange)
    GLMakie.hlines!(section_plot.ax_deviation, section_plot.obs_deviation_max, linestyle = :dash, color = :red)


    section_plot.txt_deviation_min = GLMakie.text!(section_plot.ax_deviation, 
                                                Point(30, 55),#Point(xs[end-15], observer_min[]),
                                                text = "Min → $(round(section_plot.obs_deviation_min[], digits=2))mm",
                                                align = (:left, :bottom),
                                                color = :orange)
    
    section_plot.txt_deviation_max = GLMakie.text!(section_plot.ax_deviation, 
                                                Point(35, 55),#Point(xs[end-15], observer_max[]),
                                                text = "Max → $(round(section_plot.obs_deviation_max[], digits=2))mm",
                                                align = (:left, :bottom),
                                                color = :red)  

    lift(section_plot.obs_deviation_min ) do val
        section_plot.txt_deviation_min.text = "Min → $(round(val, digits=2))mm"
    end

    lift(section_plot.obs_deviation_max) do val
        section_plot.txt_deviation_max.text = "Max → $(round(val, digits=2))mm"
    end

end



function deviation_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_deviation_surface = GLMakie.Axis3(fig[row, col],
                                                    xlabel = "θx in [°]", 
                                                    ylabel = "θz in [°]",
                                                    zlabel = "deviation in [mm]",
                                                    zticks = -500:100:500, 
                                                    title = "Ackermann deviation surface",) #
    #section_plot.ax_deviation_surface.aspect = :data
    section_plot.ax_deviation_surface.aspect = (1, 1, 1)
    section_plot.ax_deviation_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(section_plot.ax_deviation_surface, 0, 20)
    GLMakie.ylims!(section_plot.ax_deviation_surface, 0, 40)
    GLMakie.zlims!(section_plot.ax_deviation_surface, -500, 500)

    ############| Ackermannratio Data

    deviation_surface = ackermann_deviation_surface(chassis, steering, suspension, (θx_max,θy,θz_max))


    ############| Ackermannratio Observervar 
    section_plot.obs_deviation_surface = Observable(deviation_surface)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(section_plot.ax_deviation_surface, 
                        0.0:1.0:θx_max, 
                        0.0:1.0:θz_max, 
                        section_plot.obs_deviation_surface; 
                        colormap = :darkterrain)

    # XY-Ebene (z = 0) farblich hervorheben – ohne Farbmuster
    x_vals = 0.0:1.0:θx_max
    y_vals = 0.0:1.0:θz_max
    x_grid = repeat(collect(x_vals)', length(y_vals), 1)
    y_grid = repeat(collect(y_vals), 1, length(x_vals))
    z_grid = fill(0.0, size(x_grid))  # Z = 0 → XY-Ebene

    GLMakie.surface!(section_plot.ax_deviation_surface,
                        x_grid,
                        y_grid,
                        z_grid,
                        color = :red,
                        transparency = true,
                        alpha = 0.3)
    
end








"""
    update_geometry!(θ, section_plot, steering, suspension)

Updates the visual representation of the steering geometry based on the given steering angles.

# Arguments
- `θ`: A tuple `(θx, θy, θz)` representing the current steering angles.
- `section_plot`: A `PlotSection` object containing the observables used for plotting geometry.
- `steering`: The `Steering` object, containing vectors and joint positions for the steering system.
- `suspension`: The `Suspension` model, used during steering kinematics update.

# Description
This function:
- Calls `steeringkinematicsMOVED!` to update the internal state of the `steering` and `suspension` systems based on the input angles.
- Recalculates the positions of key components:
  - The rotational reference vectors
  - Left and right steering linkages
  - Stationary suspension parts
- Updates the corresponding `Observable`s in `section_plot`, triggering re-rendering of the visual geometry in the GUI.

This function is typically called reactively whenever user input modifies the steering angle configuration.

# Returns
Nothing. Modifies `section_plot` and the steering state in-place.
"""
function update_geometry!(θ, section_plot, steering, suspension)
    MMK.update!(θ, steering, suspension)
    # 
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

    # stationary
    stationary = [Point3f(steering.wishbone_ucs_position[1]...),
                        Point3f(steering.wishbone_ucs_position[2]...)]
                        
     # lower_wishbone

    conversion = (i,pos) -> steering.wishbone_ucs_position[i] + pos

    left_lower_wishbone_axis = [Point3f(steering.wishbone_ucs_position[1]...),
                                Point3f(conversion(1, suspension.lowerwishbone[1].bearing_front)...)]

    right_lower_wishbone_axis = [Point3f(steering.wishbone_ucs_position[2]...),
                                Point3f(conversion(2, suspension.lowerwishbone[2].bearing_front)...)]

    left_upper_wishbone_axis = [Point3f(conversion(1, suspension.upperwishbone[1].bearing_rear)...),
                                Point3f(conversion(1, suspension.upperwishbone[1].bearing_front)...)]

    right_upper_wishbone_axis = [Point3f(conversion(2, suspension.upperwishbone[2].bearing_rear)...),
                                Point3f(conversion(2, suspension.upperwishbone[2].bearing_front)...)]              

    left_wishbone_sphere_joint = [Point3f(conversion(1, suspension.lowerwishbone[1].sphere_joint)...),
                                   Point3f(conversion(1, suspension.upperwishbone[1].sphere_joint)...)]

    right_wishbone_sphere_joint = [Point3f(conversion(2, suspension.lowerwishbone[2].sphere_joint .*[1.0, -1.0, 1.0])...),
                                   Point3f(conversion(2, suspension.upperwishbone[2].sphere_joint .*[1.0, -1.0, 1.0])...)]


    left_lower_wishbone = [Point3f(conversion(1, suspension.lowerwishbone[1].bearing_rear + [suspension.lowerwishbone[1].distance_to_joint_x, 0, 0])...),
                            Point3f(conversion(1, suspension.lowerwishbone[1].sphere_joint)...)]   

    left_upper_wishbone = [Point3f(conversion(1, suspension.upperwishbone[1].bearing_rear + [suspension.upperwishbone[1].distance_to_joint_x, 0, 0] )...),
                            Point3f(conversion(1, suspension.upperwishbone[1].sphere_joint)...)]                     

    right_lower_wishbone = [Point3f(conversion(2, suspension.lowerwishbone[2].bearing_rear + [suspension.lowerwishbone[2].distance_to_joint_x, 0, 0] )...),
                            Point3f(conversion(2, suspension.lowerwishbone[2].sphere_joint.*[1.0, -1.0, 1.0])...)] 

    right_upper_wishbone = [Point3f(conversion(2, suspension.upperwishbone[2].bearing_rear + [suspension.upperwishbone[2].distance_to_joint_x, 0, 0] )...),
                            Point3f(conversion(2, suspension.upperwishbone[2].sphere_joint.*[1.0, -1.0, 1.0])...)] 


    # Damper

    left_damper = [Point3f(conversion(1,  suspension.damper[1].upper_fixture)...),
                         Point3f(conversion(1, suspension.damper[1].lower_fixture)...)]

    right_damper = [Point3f(conversion(2, suspension.damper[2].upper_fixture .*[1.0, -1.0, 1.0])...),
                         Point3f(conversion(2, suspension.damper[2].lower_fixture .*[1.0, -1.0, 1.0])...)]


    section_plot.obs_rotation[] = rotational_coponent
    section_plot.obs_geom_left[] = left_steering_connections
    section_plot.obs_geom_right[] = right_steering_connections
    section_plot.obs_stationary[] = stationary
    section_plot.obs_left_lower_wishbone_axis[] = left_lower_wishbone_axis
    section_plot.obs_right_lower_wishbone_axis[] = right_lower_wishbone_axis
    section_plot.obs_left_upper_wishbone_axis[] = left_upper_wishbone_axis
    section_plot.obs_right_upper_wishbone_axis[] = right_upper_wishbone_axis
    section_plot.obs_left_wishbone_sphere_joint[] = left_wishbone_sphere_joint
    section_plot.obs_right_wishbone_sphere_joint[] = right_wishbone_sphere_joint

    section_plot.obs_left_lower_wishbone[] = left_lower_wishbone
    section_plot.obs_left_upper_wishbone[] = left_upper_wishbone
    section_plot.obs_right_lower_wishbone[] = right_lower_wishbone
    section_plot.obs_right_upper_wishbone[] = right_upper_wishbone

    section_plot.obs_left_damper[] = left_damper
    section_plot.obs_right_damper[] = right_damper


end 





"""
    θ_vs_δ_plot!(fig, section_plot, θ_max, steering)

Creates a 3D surface plot showing the relationship between steering input angles and wheel angles.

# Arguments
- `fig`: A `Figure` object (Makie layout container) where the plot will be drawn.
- `section_plot`: A `PlotSection` object used to store axes and observables for the plot.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` representing the upper bounds of steering input angles.
- `steering`: The `Steering` object containing current steering angles and computation functions.

# Description
This function:
- Initializes a `GLMakie.Axis3` axis to plot the steering vs. wheel angle surfaces.
- Uses `ax_θ_vs_δi` and `ax_θ_vs_δo` to compute the inner and outer wheel steering angles across a range of `θx` and `θz`.
- Stores the generated surface data in `Observable`s to allow reactive updates.
- Draws both surfaces into the 3D plot using distinct colormaps (`:darkterrain` and `:viridis`).

This visualization helps to assess Ackermann steering behavior and the impact of steering angles on wheel geometry.

# Returns
Nothing. Modifies `section_plot` in-place by adding plot objects and data observables.
"""
function θ_vs_δ_plot!(fig, section_plot, θ_max, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]


    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_θ_vs_δ_surface = GLMakie.Axis3(fig[row, col],
                                            xlabel = "θx in [°]", 
                                            ylabel = "θz in [°]",
                                            zlabel = "θ wheel in [°]",
                                            zticks = 0:10:100, 
                                            title = "Steering vs. wheel angles",) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_θ_vs_δ_surface.aspect = (1, 1, 1)
    section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(section_plot.ax_θ_vs_δ_surface, 0, 15)
    GLMakie.ylims!(section_plot.ax_θ_vs_δ_surface, 0, 30)
    GLMakie.zlims!(section_plot.ax_θ_vs_δ_surface, 0, 105)

    ############| Ackermannratio Data

    θ_vs_δi_surface = ax_θ_vs_δi(steering, suspension, (θx_max, θy, θz_max))
    θ_vs_δo_surface = ax_θ_vs_δo(steering, suspension, (θx_max, θy, θz_max))


    ############| Ackermannratio Observervar 
    section_plot.obs_θ_vs_δi_surface = Observable(θ_vs_δi_surface)
    section_plot.obs_θ_vs_δo_surface = Observable(θ_vs_δo_surface)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(section_plot.ax_θ_vs_δ_surface , 0.0:1.0:θx_max, 0.0:1.0:θz_max, section_plot.obs_θ_vs_δi_surface; colormap = :darkterrain)
    GLMakie.surface!(section_plot.ax_θ_vs_δ_surface , 0.0:1.0:θx_max, 0.0:1.0:θz_max, section_plot.obs_θ_vs_δo_surface; colormap = :viridis)
    
end



function compr_vs_δ_plot!(fig, section_plot, steering, suspension)

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]


    θx = steering.θx
    θy = steering.θy
    θz = steering.θz
    ############| Ackermannratio Scene

    section_plot.ax_compr_vs_δ = GLMakie.Axis3(fig[row, col],
                                            xlabel = "compression left in [%]", 
                                            ylabel = "compression right in [%]",
                                            zlabel = "δi in [°]",
                                            zticks = 0:5:70, 
                                            title = "compression vs. wheel angles",) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_compr_vs_δ.aspect = (1, 1, 1)
    section_plot.ax_compr_vs_δ.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(section_plot.ax_compr_vs_δ, 0, 100)
    GLMakie.ylims!(section_plot.ax_compr_vs_δ, 0, 100)
    GLMakie.zlims!(section_plot.ax_compr_vs_δ, 0, 70)

    ############| compression vs δi Data

    compr_vs_δi, compr_vs_δo = compr_vs_δ((θx, θy, θz), steering, suspension)


    ############| Ackermannratio Observervar 
    section_plot.obs_compr_vs_δi = Observable(compr_vs_δi)
    section_plot.obs_compr_vs_δo = Observable(compr_vs_δo)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(section_plot.ax_compr_vs_δ , 1.0:1.0:100, 1.0:1.0:100, section_plot.obs_compr_vs_δi; colormap = :darkterrain)
    GLMakie.surface!(section_plot.ax_compr_vs_δ , 1.0:1.0:100, 1.0:1.0:100, section_plot.obs_compr_vs_δo; colormap = :darkterrain)
    
end