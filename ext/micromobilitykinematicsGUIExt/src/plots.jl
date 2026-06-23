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
                            ylabel = "radius in [m]", 
                            title = "Radii for (θx, θy, θz) = (0,0,0)",
                            yticks = 0:5:40) 
    
    set_axis_visible!(section_plot.ax_radii, false)

    # Limits
    GLMakie.xlims!(section_plot.ax_radii, 0, θz_max)
    GLMakie.ylims!(section_plot.ax_radii, 0, 20)

    #GLMakie.autolimits!(ax_radii)

    ############| Radii θz Data
    radii_θz = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension) ./ 1000.0

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
                                            ylabel = "Ackermann ratio [%]", 
                                            title = ackermann_ratio_title(θx, θy, θz), 
                                            xticks = 0:5:40,
                                            yticks = 30:5:100)

    set_axis_visible!(section_plot.ax_ratio, false)

    # Limits
    GLMakie.xlims!(section_plot.ax_ratio, 0, θz_max)
    GLMakie.ylims!(section_plot.ax_ratio, 30, 105)

    ############| Ackermannratio Data

    ratio = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)

    ############| Ackermannratio Observervar 

    section_plot.obs_ratio_θz = Observable(ratio)
    section_plot.obs_ratio_min = Observable(finite_minimum(ratio))
    section_plot.obs_ratio_max = Observable(finite_maximum(ratio))
    set_ratio_ylims!(section_plot.ax_ratio, ratio)



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

    on(section_plot.obs_ratio_θz) do values
        set_ratio_ylims!(section_plot.ax_ratio, values)
    end

end

function ratio_θx_plot!(fig, section_plot, θ_max, chassis, steering, suspension)

    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θy = steering.θy
    θz = steering.θz

    section_plot.ax_ratio_θx = GLMakie.Axis(fig[row, col],
                                            xlabel = "θx in [°]",
                                            ylabel = "Ackermann ratio [%]",
                                            title = ackermann_ratio_θx_title(θy, θz),
                                            xticks = 0:5:θx_max,
                                            yticks = 30:5:100)

    set_axis_visible!(section_plot.ax_ratio_θx, false)

    GLMakie.xlims!(section_plot.ax_ratio_θx, 0, θx_max)
    GLMakie.ylims!(section_plot.ax_ratio_θx, 30, 105)

    ratio = ackermannratio_θx(θx_max, θy, θz, chassis, steering, suspension)

    section_plot.obs_ratio_θx = Observable(ratio)
    section_plot.obs_ratio_θx_min = Observable(finite_minimum(ratio))
    section_plot.obs_ratio_θx_max = Observable(finite_maximum(ratio))
    set_ratio_ylims!(section_plot.ax_ratio_θx, ratio)

    xs = [θx for θx in 0.0:1.0:θx_max]

    GLMakie.lines!(section_plot.ax_ratio_θx, xs, section_plot.obs_ratio_θx)
    GLMakie.hlines!(section_plot.ax_ratio_θx, section_plot.obs_ratio_θx_min, linestyle = :dash, color = :orange)
    GLMakie.hlines!(section_plot.ax_ratio_θx, section_plot.obs_ratio_θx_max, linestyle = :dash, color = :red)

    section_plot.txt_ratio_θx_min = GLMakie.text!(section_plot.ax_ratio_θx,
                                                    Point(max(θx_max - 10, 0), 55),
                                                    text = "Min → $(round(section_plot.obs_ratio_θx_min[], digits=2))%",
                                                    align = (:left, :bottom),
                                                    color = :orange)

    section_plot.txt_ratio_θx_max = GLMakie.text!(section_plot.ax_ratio_θx,
                                                    Point(max(θx_max - 5, 0), 55),
                                                    text = "Max → $(round(section_plot.obs_ratio_θx_max[], digits=2))%",
                                                    align = (:left, :bottom),
                                                    color = :red)

    lift(section_plot.obs_ratio_θx_min) do val
        section_plot.txt_ratio_θx_min.text = "Min → $(round(val, digits=2))%"
    end

    lift(section_plot.obs_ratio_θx_max) do val
        section_plot.txt_ratio_θx_max.text = "Max → $(round(val, digits=2))%"
    end

    on(section_plot.obs_ratio_θx) do values
        set_ratio_ylims!(section_plot.ax_ratio_θx, values)
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
                                                    title = ackermann_ratio_surface_title(),) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_ratio_surface.aspect = (1, 1, 1)

    set_axis_visible!(section_plot.ax_ratio_surface, false)
    

    # Limits
    GLMakie.xlims!(section_plot.ax_ratio_surface, 0, θx_max)
    GLMakie.ylims!(section_plot.ax_ratio_surface, 0, θz_max)
    GLMakie.zlims!(section_plot.ax_ratio_surface, 50, 105)

    ############| Ackermannratio Data

    ratio_surface = ackermannratio_surface(chassis, steering, suspension, (θx_max,θy,θz_max))


    ############| Ackermannratio Observervar 
    section_plot.obs_ratio_surface = Observable(ratio_surface)
    set_ratio_zlims!(section_plot.ax_ratio_surface, ratio_surface)
    on(section_plot.obs_ratio_surface) do values
        set_ratio_zlims!(section_plot.ax_ratio_surface, values)
    end

    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        section_plot.ax_ratio_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        section_plot.obs_ratio_surface;
        color = section_plot.obs_ratio_surface,
        colormap = lift(data -> ackermann_ratio_surface_colormap(data), section_plot.obs_ratio_surface),
        colorrange = lift(data -> ratio_surface_colorrange(data), section_plot.obs_ratio_surface),
    )
    
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
                                            ylabel = "Ackermann deviation [mm]", 
                                            title = "Ackermann deviation for (θx, θy, θz) = (0,0,0)", 
                                            xticks = 0:5:40,
                                            yticks = -500:100:500)

    set_axis_visible!(section_plot.ax_deviation,    false)

    # Limits
    GLMakie.xlims!(section_plot.ax_deviation, 0, θz_max)
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
    
    set_axis_visible!(section_plot.ax_deviation_surface, false)
    

    # Limits
    GLMakie.xlims!(section_plot.ax_deviation_surface, 0, θx_max)
    GLMakie.ylims!(section_plot.ax_deviation_surface, 0, θz_max)
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
                        colormap   = :reds,           # oder :viridis o.Ä.
                        transparency = true,
                        alpha      = 0.3)
    
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
                                            zlabel = "wheel angle δ in [°]",
                                            zticks = 0:10:100, 
                                            title = theta_vs_delta_title(θx_max, θy, θz_max),) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_θ_vs_δ_surface.aspect = (1, 1, 1)
    
    
    set_axis_visible!(section_plot.ax_θ_vs_δ_surface,    false)

    # Limits
    GLMakie.xlims!(section_plot.ax_θ_vs_δ_surface, 0, θx_max)
    GLMakie.ylims!(section_plot.ax_θ_vs_δ_surface, 0, θz_max)
    GLMakie.zlims!(section_plot.ax_θ_vs_δ_surface, 0, 105)

    ############| Ackermannratio Data

    θ_vs_δi_surface = ax_θ_vs_δi(steering, suspension, (θx_max, θy, θz_max))
    θ_vs_δo_surface = ax_θ_vs_δo(steering, suspension, (θx_max, θy, θz_max))


    ############| Ackermannratio Observervar 
    section_plot.obs_θ_vs_δi_surface = Observable(θ_vs_δi_surface)
    section_plot.obs_θ_vs_δo_surface = Observable(θ_vs_δo_surface)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        section_plot.ax_θ_vs_δ_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        section_plot.obs_θ_vs_δi_surface;
        color = fill(1.0, size(θ_vs_δi_surface)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.78,
    )
    GLMakie.surface!(
        section_plot.ax_θ_vs_δ_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        section_plot.obs_θ_vs_δo_surface;
        color = fill(1.0, size(θ_vs_δo_surface)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.70,
    )
    
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
                                            xlabel = "left compression in [%]", 
                                            ylabel = "right compression in [%]",
                                            zlabel = "wheel angle δ in [°]",
                                            zticks = 0:5:70, 
                                            title = compr_vs_delta_title(),) #
    #section_plot.ax_ratio_surface.aspect = :data
    section_plot.ax_compr_vs_δ.aspect = (1, 1, 1)
    
    set_axis_visible!(section_plot.ax_compr_vs_δ,    false)

    # Limits
    GLMakie.xlims!(section_plot.ax_compr_vs_δ, 0, 100)
    GLMakie.ylims!(section_plot.ax_compr_vs_δ, 0, 100)

    ############| compression vs δi Data

    compr_vs_δi, compr_vs_δo = compr_vs_δ((θx, θy, θz), steering, suspension)
    compression_range = range(0.0, 100.0; length = size(compr_vs_δi, 1))
    set_compr_vs_delta_zlims!(section_plot.ax_compr_vs_δ, compr_vs_δi, compr_vs_δo)


    ############| Ackermannratio Observervar 
    section_plot.obs_compr_vs_δi = Observable(compr_vs_δi)
    section_plot.obs_compr_vs_δo = Observable(compr_vs_δo)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        section_plot.ax_compr_vs_δ,
        compression_range,
        compression_range,
        section_plot.obs_compr_vs_δi;
        color = fill(1.0, size(compr_vs_δi)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.78,
    )

    GLMakie.surface!(
        section_plot.ax_compr_vs_δ,
        compression_range,
        compression_range,
        section_plot.obs_compr_vs_δo;
        color = fill(1.0, size(compr_vs_δo)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.70,
    )
    
end

function left_wheel_delta_plot!(fig, section_plot, θ_max, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θx = steering.θx
    θy = steering.θy
    right_compression = suspension.damper[2].compression

    section_plot.ax_left_wheel_delta = GLMakie.Axis3(fig[row, col],
                                            xlabel = "left compression in [%]",
                                            ylabel = "θz in [°]",
                                            zlabel = "left wheel Δδ in [°]",
                                            title = left_wheel_delta_title(θx, θy, right_compression, θz_max),)
    section_plot.ax_left_wheel_delta.aspect = (1, 1, 1)

    set_axis_visible!(section_plot.ax_left_wheel_delta, false)

    GLMakie.xlims!(section_plot.ax_left_wheel_delta, 0, 100)
    GLMakie.ylims!(section_plot.ax_left_wheel_delta, 0, θz_max)

    delta_left = left_wheel_delta_vs_compression_θz(
        θx,
        θy,
        θz_max,
        steering,
        suspension;
        fixed_right_compression = right_compression,
    )
    compression_range = range(0.0, 100.0; length = size(delta_left, 1))
    θz_range = range(0.0, θz_max; length = size(delta_left, 2))

    section_plot.obs_left_wheel_delta = Observable(delta_left)
    set_left_wheel_delta_zlims!(section_plot.ax_left_wheel_delta, delta_left)

    GLMakie.surface!(
        section_plot.ax_left_wheel_delta,
        compression_range,
        θz_range,
        section_plot.obs_left_wheel_delta;
        colormap = :viridis,
    )

    nothing
end

function wheel_center_path_plot!(fig, section_plot, steering, suspension)
    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    section_plot.ax_wheel_center_path = GLMakie.Axis3(fig[row, col],
                                            xlabel = "x in [mm]",
                                            ylabel = "y in [mm]",
                                            zlabel = "z in [mm]",
                                            title = wheel_center_path_title(),)
    section_plot.ax_wheel_center_path.aspect = (1, 1, 1)

    set_axis_visible!(section_plot.ax_wheel_center_path, false)

    compression_values, left_path, right_path = wheel_center_path(steering, suspension)

    section_plot.obs_wheel_center_left = Observable(left_path)
    section_plot.obs_wheel_center_right = Observable(right_path)
    set_wheel_center_path_limits!(section_plot.ax_wheel_center_path, left_path, right_path)

    GLMakie.lines!(section_plot.ax_wheel_center_path, section_plot.obs_wheel_center_left; color = :royalblue, linewidth = 3)
    GLMakie.lines!(section_plot.ax_wheel_center_path, section_plot.obs_wheel_center_right; color = :darkorange, linewidth = 3)
    GLMakie.scatter!(section_plot.ax_wheel_center_path, section_plot.obs_wheel_center_left; color = :royalblue, markersize = 6)
    GLMakie.scatter!(section_plot.ax_wheel_center_path, section_plot.obs_wheel_center_right; color = :darkorange, markersize = 6)

    on(section_plot.obs_wheel_center_left) do values
        set_wheel_center_path_limits!(section_plot.ax_wheel_center_path, values, section_plot.obs_wheel_center_right[])
    end

    on(section_plot.obs_wheel_center_right) do values
        set_wheel_center_path_limits!(section_plot.ax_wheel_center_path, section_plot.obs_wheel_center_left[], values)
    end

    nothing
end

function wheel_center_surface_plot!(fig, section_plot, θ_max, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    θx = steering.θx
    θy = steering.θy

    section_plot.ax_wheel_center_surface = GLMakie.Axis3(fig[row, col],
                                            xlabel = "x in [mm]",
                                            ylabel = "y in [mm]",
                                            zlabel = "z in [mm]",
                                            title = wheel_center_surface_title(θx, θy, θz_max),)
    section_plot.ax_wheel_center_surface.aspect = (1, 1, 1)

    set_axis_visible!(section_plot.ax_wheel_center_surface, false)

    (
        compression_values,
        θz_values,
        left_x,
        left_y,
        left_z,
        right_x,
        right_y,
        right_z,
    ) = wheel_center_surface(θx, θy, θz_max, steering, suspension)

    section_plot.obs_wheel_center_surface_left_x = Observable(left_x)
    section_plot.obs_wheel_center_surface_left_y = Observable(left_y)
    section_plot.obs_wheel_center_surface_left_z = Observable(left_z)
    section_plot.obs_wheel_center_surface_right_x = Observable(right_x)
    section_plot.obs_wheel_center_surface_right_y = Observable(right_y)
    section_plot.obs_wheel_center_surface_right_z = Observable(right_z)

    set_wheel_center_surface_limits!(
        section_plot.ax_wheel_center_surface,
        (left_x, left_y, left_z),
        (right_x, right_y, right_z),
    )

    GLMakie.surface!(
        section_plot.ax_wheel_center_surface,
        section_plot.obs_wheel_center_surface_left_x,
        section_plot.obs_wheel_center_surface_left_y,
        section_plot.obs_wheel_center_surface_left_z;
        color = fill(1.0, size(left_z)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.55,
    )

    GLMakie.surface!(
        section_plot.ax_wheel_center_surface,
        section_plot.obs_wheel_center_surface_right_x,
        section_plot.obs_wheel_center_surface_right_y,
        section_plot.obs_wheel_center_surface_right_z;
        color = fill(1.0, size(right_z)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.50,
    )

    nothing
end

function track_width_plot!(fig, section_plot, steering, suspension)
    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    section_plot.ax_track_width = GLMakie.Axis(fig[row, col],
                                            xlabel = "symmetric compression in [%]",
                                            ylabel = "track width in [mm]",
                                            title = track_width_title(),
                                            xticks = 0:10:100)

    set_axis_visible!(section_plot.ax_track_width, false)

    compression_values, track_width = track_width_over_compression(steering, suspension)
    section_plot.obs_track_width = Observable(track_width)
    set_line_ylims!(section_plot.ax_track_width, track_width; lower_floor = 0.0, min_span = 1.0)
    GLMakie.xlims!(section_plot.ax_track_width, 0, 100)

    GLMakie.lines!(section_plot.ax_track_width, compression_values, section_plot.obs_track_width; color = :seagreen, linewidth = 3)

    on(section_plot.obs_track_width) do values
        set_line_ylims!(section_plot.ax_track_width, values; lower_floor = 0.0, min_span = 1.0)
    end

    nothing
end

function motion_ratio_plot!(fig, section_plot, steering, suspension)
    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    section_plot.ax_motion_ratio = GLMakie.Axis(fig[row, col],
                                            xlabel = "symmetric compression in [%]",
                                            ylabel = motion_ratio_ylabel(),
                                            title = motion_ratio_title(),
                                            xticks = 0:10:100)

    set_axis_visible!(section_plot.ax_motion_ratio, false)

    compression_values, motion_ratio = damper_motion_ratio(steering, suspension)
    section_plot.obs_motion_ratio = Observable(motion_ratio)
    set_line_ylims!(section_plot.ax_motion_ratio, motion_ratio; lower_floor = 0.0, min_span = 0.1)
    GLMakie.xlims!(section_plot.ax_motion_ratio, 0, 100)

    GLMakie.lines!(section_plot.ax_motion_ratio, compression_values, section_plot.obs_motion_ratio; color = :royalblue, linewidth = 3)

    on(section_plot.obs_motion_ratio) do values
        set_line_ylims!(section_plot.ax_motion_ratio, values; lower_floor = 0.0, min_span = 0.1)
    end

    nothing
end

function roll_kinematics_plot!(fig, section_plot, chassis, steering, suspension)
    slot = section_plot.slot
    row = slot[1]
    col = slot[2]

    roll_layout = GridLayout(tellheight = false)
    fig[row, col] = roll_layout

    roll_xlabel = "roll state: left compression / right rebound [%]"
    section_plot.ax_roll_camber = GLMakie.Axis(roll_layout[1, 1],
                                            xlabel = roll_xlabel,
                                            ylabel = "camber [deg]",
                                            title = roll_camber_title(),
                                            xticks = 0:20:100)

    section_plot.ax_roll_wheel_angle = GLMakie.Axis(roll_layout[1, 2],
                                            xlabel = roll_xlabel,
                                            ylabel = "wheel angle δ [deg]",
                                            title = roll_wheel_angle_title(),
                                            xticks = 0:20:100)

    section_plot.ax_roll_track_width = GLMakie.Axis(roll_layout[2, 1],
                                            xlabel = roll_xlabel,
                                            ylabel = "track width [mm]",
                                            title = roll_track_width_title(),
                                            xticks = 0:20:100)

    section_plot.ax_roll_ackermann_deviation = GLMakie.Axis(roll_layout[2, 2],
                                            xlabel = roll_xlabel,
                                            ylabel = "Ackermann ratio [%]",
                                            title = roll_ackermann_ratio_title(),
                                            xticks = 0:20:100)

    set_axis_visible!(section_plot.ax_roll_camber, false)
    set_axis_visible!(section_plot.ax_roll_wheel_angle, false)
    set_axis_visible!(section_plot.ax_roll_track_width, false)
    set_axis_visible!(section_plot.ax_roll_ackermann_deviation, false)

    θ = (steering.θx, steering.θy, steering.θz)
    (
        roll_values,
        left_camber,
        right_camber,
        left_wheel_angle,
        right_wheel_angle,
        track_width,
        ackermann_ratio_values,
    ) = roll_kinematics(θ, chassis, steering, suspension)

    section_plot.obs_roll_left_camber = Observable(left_camber)
    section_plot.obs_roll_right_camber = Observable(right_camber)
    section_plot.obs_roll_left_wheel_angle = Observable(left_wheel_angle)
    section_plot.obs_roll_right_wheel_angle = Observable(right_wheel_angle)
    section_plot.obs_roll_track_width = Observable(track_width)
    section_plot.obs_roll_ackermann_deviation = Observable(ackermann_ratio_values)

    GLMakie.xlims!(section_plot.ax_roll_camber, 0, 100)
    GLMakie.xlims!(section_plot.ax_roll_wheel_angle, 0, 100)
    GLMakie.xlims!(section_plot.ax_roll_track_width, 0, 100)
    GLMakie.xlims!(section_plot.ax_roll_ackermann_deviation, 0, 100)

    set_line_ylims!(section_plot.ax_roll_camber, left_camber, right_camber; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(section_plot.ax_roll_wheel_angle, left_wheel_angle, right_wheel_angle; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(section_plot.ax_roll_track_width, track_width; lower_floor = 0.0, min_span = 1.0)
    set_ratio_ylims!(section_plot.ax_roll_ackermann_deviation, ackermann_ratio_values; signed = ackermann_ratio_signed(), lower_default = 30.0)

    GLMakie.lines!(section_plot.ax_roll_camber, roll_values, section_plot.obs_roll_left_camber; color = :royalblue, linewidth = 3)
    GLMakie.lines!(section_plot.ax_roll_camber, roll_values, section_plot.obs_roll_right_camber; color = :darkorange, linewidth = 3)
    GLMakie.lines!(section_plot.ax_roll_wheel_angle, roll_values, section_plot.obs_roll_left_wheel_angle; color = :royalblue, linewidth = 3)
    GLMakie.lines!(section_plot.ax_roll_wheel_angle, roll_values, section_plot.obs_roll_right_wheel_angle; color = :darkorange, linewidth = 3)
    GLMakie.lines!(section_plot.ax_roll_track_width, roll_values, section_plot.obs_roll_track_width; color = :seagreen, linewidth = 3)
    GLMakie.lines!(section_plot.ax_roll_ackermann_deviation, roll_values, section_plot.obs_roll_ackermann_deviation; color = :firebrick, linewidth = 3)

    nothing
end
