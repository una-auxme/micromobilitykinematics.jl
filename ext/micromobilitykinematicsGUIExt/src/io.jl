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
    ax_geom = GLMakie.Axis3(fig[1:2, 1:3],
                            xlabel = "x in [mm]", 
                            ylabel = "y in [mm]",
                            zlabel = "z in [mm]", 
                            title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)",) #
    ax_geom.aspect = :data




    

    # Limits
    GLMakie.xlims!(ax_geom, -200, 50)
    GLMakie.ylims!(ax_geom, -300, 300)
    GLMakie.zlims!(ax_geom, -200, 50)


        ############| Geometry Data
    rotation = [Point3f([0,0,0]),
                                Point3f(steering.vec_x_rotational...),
                                Point3f(steering.vec_z_rotational...)]

    geom_left = [Point3f(steering.vec_z_rotational...),
                                    Point3f(steering.sphere_joints[1]...),
                                    Point3f(steering.circle_joints[1]...),
                                    Point3f(steering.track_lever_mounting_points_ucs[1]...)]

    geom_right = [Point3f(steering.vec_z_rotational...),
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



    ############| Geometry Ploting   
    GLMakie.scatter!(ax_geom, rotation, markersize=10)
    GLMakie.scatter!(ax_geom, geom_left, markersize=10)
    GLMakie.scatter!(ax_geom, geom_right, markersize=10)
    GLMakie.scatter!(ax_geom, stationary, markersize=10)
    GLMakie.scatter!(ax_geom, left_lower_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_lower_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, left_upper_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_upper_wishbone_axis, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, left_wishbone_sphere_joint, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_wishbone_sphere_joint, markersize=10; color = :black)

    GLMakie.scatter!(ax_geom, left_lower_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, left_upper_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_lower_wishbone, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_upper_wishbone, markersize=10; color = :black)

    GLMakie.scatter!(ax_geom, left_damper, markersize=10; color = :black)
    GLMakie.scatter!(ax_geom, right_damper, markersize=10; color = :black)


    GLMakie.lines!(ax_geom, rotation)
    GLMakie.lines!(ax_geom, geom_left)
    GLMakie.lines!(ax_geom, geom_right)
    GLMakie.lines!(ax_geom, left_lower_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, right_lower_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, left_upper_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, right_upper_wishbone_axis; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, left_wishbone_sphere_joint; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, right_wishbone_sphere_joint; linestyle = :dash, color = :black)

    GLMakie.lines!(ax_geom, left_lower_wishbone; color = :black)
    GLMakie.lines!(ax_geom, left_upper_wishbone; color = :black)
    GLMakie.lines!(ax_geom, right_lower_wishbone; color = :black)
    GLMakie.lines!(ax_geom, right_upper_wishbone; color = :black)

    GLMakie.lines!(ax_geom, left_damper; linestyle = :dash, color = :black)
    GLMakie.lines!(ax_geom, right_damper; linestyle = :dash, color = :black)





    return fig
end 



function radii_plot(θx,θy,θz_max,chassis, steering, suspension)

    fig = GLMakie.Figure(size = (900, 600))

    ############| Radii θz Scene
    ax_radii = GLMakie.Axis(fig[1:2, 1:3], 
                            xlabel = "θz in [°]", 
                            ylabel = "radius in [m]", 
                            title = "Radii for (θx, θy, θz) = (0,0,0)",
                            yticks = 0:5:40) 
    #ax_radii.blockscene.visible[] = false
    # Limits
    GLMakie.xlims!(ax_radii, 0, θz_max)
    GLMakie.ylims!(ax_radii, 0, 20)

    #GLMakie.autolimits!(ax_radii)

    ############| Radii θz Data
    radii_θz = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension) ./ 1000.0


    ############| Radii θz Ploting  
    xs = [θz for θz in 0.0:1.0:θz_max] 

    GLMakie.lines!(ax_radii, xs, radii_θz)

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
function ackermannratio_θz_plot(θx,θy,θz_max,chassis, steering, suspension; signed = ackermann_ratio_signed())

    ratio = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension; signed = signed)

    fig = GLMakie.Figure(size = (900, 600))

    ax_ratio = GLMakie.Axis(fig[1:2, 1:3], 
                            xlabel = "θz in [°]", 
                            ylabel = "Ackermann ratio [%]", 
                            title = ackermann_ratio_title(θx, θy, 0; signed = signed), 
                            xticks = 0:5:40,
                            yticks = 50:5:100)

    # Limits
    GLMakie.xlims!(ax_ratio, 0, θz_max)
    set_ratio_ylims!(ax_ratio, ratio; signed = signed, lower_default = 50.0)


    min = finite_minimum(ratio)
    max = finite_maximum(ratio)

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
                                text = "Max → $(round(max, digits=2))%",
                                align = (:left, :bottom),
                                color = :red)  

    return fig

end

function ackermannratio_θx_plot(θx_max, θy, θz, chassis, steering, suspension; signed = ackermann_ratio_signed())

    ratio = ackermannratio_θx(θx_max, θy, θz, chassis, steering, suspension; signed = signed)

    fig = GLMakie.Figure(size = (900, 600))

    ax_ratio = GLMakie.Axis(fig[1:2, 1:3],
                            xlabel = "θx in [°]",
                            ylabel = "Ackermann ratio [%]",
                            title = ackermann_ratio_θx_title(θy, θz; signed = signed),
                            xticks = 0:5:θx_max,
                            yticks = 50:5:100)

    GLMakie.xlims!(ax_ratio, 0, θx_max)
    set_ratio_ylims!(ax_ratio, ratio; signed = signed, lower_default = 50.0)

    min_ratio = finite_minimum(ratio)
    max_ratio = finite_maximum(ratio)

    xs = [θx for θx in 0:1:θx_max]
    GLMakie.lines!(ax_ratio, xs, ratio)

    GLMakie.hlines!(ax_ratio, min_ratio, linestyle = :dash, color = :orange)
    GLMakie.hlines!(ax_ratio, max_ratio, linestyle = :dash, color = :red)

    GLMakie.text!(ax_ratio,
                    Point(max(θx_max - 10, 0), 55),
                    text = "Min → $(round(min_ratio, digits=2))%",
                    align = (:left, :bottom),
                    color = :orange)

    GLMakie.text!(ax_ratio,
                    Point(max(θx_max - 5, 0), 55),
                    text = "Max → $(round(max_ratio, digits=2))%",
                    align = (:left, :bottom),
                    color = :red)

    return fig

end


function ratio_surface_plot(θy, θ_max, chassis, steering, suspension; signed = ackermann_ratio_signed())
    θx_max, θy_max, θz_max = θ_max

    fig = GLMakie.Figure(size = (900, 600))

    ############| Ackermannratio Scene

    ax_ratio_surface = GLMakie.Axis3(fig[1:2, 1:3],
                                        xlabel = "θx in [°]", 
                                        ylabel = "θz in [°]",
                                        zlabel = "ratio in [%]",
                                        zticks = 50:10:100, 
                                        title = ackermann_ratio_surface_title(; signed = signed),) #
    #section_plot.ax_ratio_surface.aspect = :data
    #ax_ratio_surface.aspect = (1, 1, 1)
    #ax_ratio_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(ax_ratio_surface, 0, θx_max)
    GLMakie.ylims!(ax_ratio_surface, 0, θz_max)
    GLMakie.zlims!(ax_ratio_surface, 50, 105)

    ############| Ackermannratio Data

    ratio_surface = ackermannratio_surface(chassis, steering, suspension, (θx_max,θy,θz_max); signed = signed)
    set_ratio_zlims!(ax_ratio_surface, ratio_surface; signed = signed)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        ax_ratio_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        ratio_surface;
        color = ratio_surface,
        colormap = ackermann_ratio_surface_colormap(ratio_surface; signed = signed),
        colorrange = ratio_surface_colorrange(ratio_surface; signed = signed),
    )

    return fig
    
end



function deviation_plot(θx,θy,θz_max,chassis, steering, suspension)



    fig = GLMakie.Figure(size = (900, 600))

    ############| Ackermannratio Scene

    ax_deviation = GLMakie.Axis(fig[1:2, 1:3], 
                                    xlabel = "θz in [°]", 
                                    ylabel = "Ackermann deviation [mm]", 
                                    title = "Ackermann deviation for (θx, θy, θz) = (0,0,0)", 
                                    xticks = 0:5:40,
                                    yticks = -500:100:500)

    ax_deviation.blockscene.visible[] = false


    # Limits
    GLMakie.xlims!(ax_deviation, 0, θz_max)
    GLMakie.ylims!(ax_deviation, -500,500 )

    ############| Ackermannratio Data

    deviation_θz = ackermann_deviation_θz(θx,θy,θz_max,chassis, steering, suspension)
    deviation_min = minimum(deviation_θz)
    deviation_max = maximum(deviation_θz)

    ############| Ackermannratio Ploting  
    xs = [θz for θz in 0.0:1.0:θz_max] 

    GLMakie.lines!(ax_deviation, xs, deviation_θz)

    #
    GLMakie.hlines!(ax_deviation, deviation_min, linestyle = :dash, color = :orange)
    GLMakie.hlines!(ax_deviation, deviation_max, linestyle = :dash, color = :red)


    txt_deviation_min = GLMakie.text!(ax_deviation, 
                                                Point(30, 55),#Point(xs[end-15], observer_min[]),
                                                text = "Min → $(round(deviation_min[], digits=2))mm",
                                                align = (:left, :bottom),
                                                color = :orange)
    
    txt_deviation_max = GLMakie.text!(ax_deviation, 
                                                Point(35, 55),#Point(xs[end-15], observer_max[]),
                                                text = "Max → $(round(deviation_max[], digits=2))mm",
                                                align = (:left, :bottom),
                                                color = :red)  

    return fig

end


function deviation_surface_plot(θy, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    fig = GLMakie.Figure(size = (900, 600))

    ############| Ackermannratio Scene

    ax_deviation_surface = GLMakie.Axis3(fig[1:2, 1:3],
                                            xlabel = "θx in [°]", 
                                            ylabel = "θz in [°]",
                                            zlabel = "deviation in [mm]",
                                            zticks = -500:100:500, 
                                            title = "Ackermann deviation surface",) #
    #section_plot.ax_deviation_surface.aspect = :data
    #section_plot.ax_deviation_surface.aspect = (1, 1, 1)
    #section_plot.ax_deviation_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(ax_deviation_surface, 0, θx_max)
    GLMakie.ylims!(ax_deviation_surface, 0, θz_max)
    GLMakie.zlims!(ax_deviation_surface, -500, 500)

    ############| Ackermannratio Data

    deviation_surface = ackermann_deviation_surface(chassis, steering, suspension, (θx_max,θy,θz_max))


    ############| Ackermannratio Ploting  

    GLMakie.surface!(ax_deviation_surface, 
                        0.0:1.0:θx_max, 
                        0.0:1.0:θz_max, 
                        deviation_surface; 
                        colormap = :darkterrain)

    # XY-Ebene (z = 0) farblich hervorheben – ohne Farbmuster
    x_vals = 0.0:1.0:θx_max
    y_vals = 0.0:1.0:θz_max
    x_grid = repeat(collect(x_vals)', length(y_vals), 1)
    y_grid = repeat(collect(y_vals), 1, length(x_vals))
    z_grid = fill(0.0, size(x_grid))  # Z = 0 → XY-Ebene

    GLMakie.surface!(ax_deviation_surface,
                        x_grid,
                        y_grid,
                        z_grid,
                        colormap = :reds,
                        transparency = true,
                        alpha = 0.3)
    
    return fig
end


function θ_vs_δ_plot(θy, θ_max, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    fig = GLMakie.Figure(size = (900, 600))

    ############| Ackermannratio Scene

    ax_θ_vs_δ_surface = GLMakie.Axis3(fig[1:2, 1:3],
                                        xlabel = "θx in [°]", 
                                        ylabel = "θz in [°]",
                                        zlabel = "wheel angle δ in [°]",
                                        zticks = 0:10:100, 
                                        title = theta_vs_delta_title(θx_max, θy, θz_max),) #
    #section_plot.ax_ratio_surface.aspect = :data
    #section_plot.ax_θ_vs_δ_surface.aspect = (1, 1, 1)
    #section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(ax_θ_vs_δ_surface, 0, θx_max)
    GLMakie.ylims!(ax_θ_vs_δ_surface, 0, θz_max)
    GLMakie.zlims!(ax_θ_vs_δ_surface, 0, 105)

    ############| Ackermannratio Data

    θ_vs_δi_surface = ax_θ_vs_δi(steering, suspension, (θx_max, θy, θz_max))
    θ_vs_δo_surface = ax_θ_vs_δo(steering, suspension, (θx_max, θy, θz_max))

    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        ax_θ_vs_δ_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        θ_vs_δi_surface;
        color = fill(1.0, size(θ_vs_δi_surface)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.78,
    )
    GLMakie.surface!(
        ax_θ_vs_δ_surface,
        0.0:1.0:θx_max,
        0.0:1.0:θz_max,
        θ_vs_δo_surface;
        color = fill(1.0, size(θ_vs_δo_surface)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.70,
    )
    
    return fig
end


function compr_vs_δ_plot(θx, θy, θz, steering, suspension)

    fig = GLMakie.Figure(size = (900, 600))


    ############| Ackermannratio Scene

    ax_compr_vs_δ = GLMakie.Axis3(fig[1:2, 1:3],
                                    xlabel = "left compression in [%]", 
                                    ylabel = "right compression in [%]",
                                    zlabel = "wheel angle δ in [°]",
                                    zticks = 0:5:70, 
                                    title = compr_vs_delta_title(),) #
    #section_plot.ax_ratio_surface.aspect = :data
    #section_plot.ax_compr_vs_δi.aspect = (1, 1, 1)
    #section_plot.ax_compr_vs_δi.blockscene.visible[] = false
    

    # Limits
    GLMakie.xlims!(ax_compr_vs_δ, 0, 100)
    GLMakie.ylims!(ax_compr_vs_δ, 0, 100)

    ############| compression vs δi Data

    compr_vs_δi, compr_vs_δo= compr_vs_δ((θx, θy, θz), steering, suspension)
    compression_range = range(0.0, 100.0; length = size(compr_vs_δi, 1))
    set_compr_vs_delta_zlims!(ax_compr_vs_δ, compr_vs_δi, compr_vs_δo)


    ############| Ackermannratio Ploting  

    GLMakie.surface!(
        ax_compr_vs_δ,
        compression_range,
        compression_range,
        compr_vs_δi;
        color = fill(1.0, size(compr_vs_δi)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.78,
    )

    GLMakie.surface!(
        ax_compr_vs_δ,
        compression_range,
        compression_range,
        compr_vs_δo;
        color = fill(1.0, size(compr_vs_δo)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.70,
    )

    return fig
    
end

function left_wheel_delta_plot(θx, θy, θz_max, steering, suspension)
    fig = GLMakie.Figure(size = (900, 600))

    right_compression = suspension.damper[2].compression

    ax_left_wheel_delta = GLMakie.Axis3(fig[1:2, 1:3],
                                    xlabel = "left compression in [%]",
                                    ylabel = "θz in [°]",
                                    zlabel = "left wheel Δδ in [°]",
                                    title = left_wheel_delta_title(θx, θy, right_compression, θz_max),)

    GLMakie.xlims!(ax_left_wheel_delta, 0, 100)
    GLMakie.ylims!(ax_left_wheel_delta, 0, θz_max)

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
    set_left_wheel_delta_zlims!(ax_left_wheel_delta, delta_left)

    GLMakie.surface!(
        ax_left_wheel_delta,
        compression_range,
        θz_range,
        delta_left;
        colormap = :viridis,
    )

    return fig
end

function wheel_center_path_plot(steering, suspension)
    fig = GLMakie.Figure(size = (900, 600))

    ax_wheel_center_path = GLMakie.Axis3(fig[1:2, 1:3],
                                    xlabel = "x in [mm]",
                                    ylabel = "y in [mm]",
                                    zlabel = "z in [mm]",
                                    title = wheel_center_path_title(),)
    ax_wheel_center_path.aspect = (1, 1, 1)

    compression_values, left_path, right_path = wheel_center_path(steering, suspension)
    set_wheel_center_path_limits!(ax_wheel_center_path, left_path, right_path)

    GLMakie.lines!(ax_wheel_center_path, left_path; color = :royalblue, linewidth = 3)
    GLMakie.lines!(ax_wheel_center_path, right_path; color = :darkorange, linewidth = 3)
    GLMakie.scatter!(ax_wheel_center_path, left_path; color = :royalblue, markersize = 6)
    GLMakie.scatter!(ax_wheel_center_path, right_path; color = :darkorange, markersize = 6)

    return fig
end

function wheel_center_surface_plot(θx, θy, θz_max, steering, suspension)
    fig = GLMakie.Figure(size = (900, 600))

    ax_wheel_center_surface = GLMakie.Axis3(fig[1:2, 1:3],
                                    xlabel = "x in [mm]",
                                    ylabel = "y in [mm]",
                                    zlabel = "z in [mm]",
                                    title = wheel_center_surface_title(θx, θy, θz_max),)
    ax_wheel_center_surface.aspect = (1, 1, 1)

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

    set_wheel_center_surface_limits!(
        ax_wheel_center_surface,
        (left_x, left_y, left_z),
        (right_x, right_y, right_z),
    )

    GLMakie.surface!(
        ax_wheel_center_surface,
        left_x,
        left_y,
        left_z;
        color = fill(1.0, size(left_z)),
        colormap = [:royalblue, :royalblue],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.55,
    )

    GLMakie.surface!(
        ax_wheel_center_surface,
        right_x,
        right_y,
        right_z;
        color = fill(1.0, size(right_z)),
        colormap = [:darkorange, :darkorange],
        colorrange = (0.0, 1.0),
        transparency = true,
        alpha = 0.50,
    )

    return fig
end

function track_width_plot(steering, suspension)
    fig = GLMakie.Figure(size = (900, 600))

    ax_track_width = GLMakie.Axis(fig[1:2, 1:3],
                                xlabel = "symmetric compression in [%]",
                                ylabel = "track width in [mm]",
                                title = track_width_title(),
                                xticks = 0:10:100)

    compression_values, track_width = track_width_over_compression(steering, suspension)
    set_line_ylims!(ax_track_width, track_width; lower_floor = 0.0, min_span = 1.0)
    GLMakie.xlims!(ax_track_width, 0, 100)

    GLMakie.lines!(ax_track_width, compression_values, track_width; color = :seagreen, linewidth = 3)

    return fig
end

function motion_ratio_plot(steering, suspension)
    fig = GLMakie.Figure(size = (900, 600))

    ax_motion_ratio = GLMakie.Axis(fig[1:2, 1:3],
                                xlabel = "symmetric compression in [%]",
                                ylabel = motion_ratio_ylabel(),
                                title = motion_ratio_title(),
                                xticks = 0:10:100)

    compression_values, motion_ratio = damper_motion_ratio(steering, suspension)
    set_line_ylims!(ax_motion_ratio, motion_ratio; lower_floor = 0.0, min_span = 0.1)
    GLMakie.xlims!(ax_motion_ratio, 0, 100)

    GLMakie.lines!(ax_motion_ratio, compression_values, motion_ratio; color = :royalblue, linewidth = 3)

    return fig
end

function roll_kinematics_plot(θ, chassis, steering, suspension; signed = ackermann_ratio_signed())
    fig = GLMakie.Figure(size = (1100, 750))
    roll_layout = GridLayout()
    fig[1, 1] = roll_layout

    roll_xlabel = "roll state: left compression / right rebound [%]"
    ax_roll_camber = GLMakie.Axis(roll_layout[1, 1],
                                xlabel = roll_xlabel,
                                ylabel = "camber [deg]",
                                title = roll_camber_title(),
                                xticks = 0:20:100)
    ax_roll_wheel_angle = GLMakie.Axis(roll_layout[1, 2],
                                xlabel = roll_xlabel,
                                ylabel = "wheel angle δ [deg]",
                                title = roll_wheel_angle_title(),
                                xticks = 0:20:100)
    ax_roll_track_width = GLMakie.Axis(roll_layout[2, 1],
                                xlabel = roll_xlabel,
                                ylabel = "track width [mm]",
                                title = roll_track_width_title(),
                                xticks = 0:20:100)
    ax_roll_ackermann_deviation = GLMakie.Axis(roll_layout[2, 2],
                                xlabel = roll_xlabel,
                                ylabel = "Ackermann ratio [%]",
                                title = roll_ackermann_ratio_title(; signed = signed),
                                xticks = 0:20:100)

    (
        roll_values,
        left_camber,
        right_camber,
        left_wheel_angle,
        right_wheel_angle,
        track_width,
        ackermann_ratio_values,
    ) = roll_kinematics(θ, chassis, steering, suspension; signed = signed)

    GLMakie.xlims!(ax_roll_camber, 0, 100)
    GLMakie.xlims!(ax_roll_wheel_angle, 0, 100)
    GLMakie.xlims!(ax_roll_track_width, 0, 100)
    GLMakie.xlims!(ax_roll_ackermann_deviation, 0, 100)

    set_line_ylims!(ax_roll_camber, left_camber, right_camber; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(ax_roll_wheel_angle, left_wheel_angle, right_wheel_angle; lower_floor = -Inf, min_span = 1.0)
    set_line_ylims!(ax_roll_track_width, track_width; lower_floor = 0.0, min_span = 1.0)
    set_ratio_ylims!(ax_roll_ackermann_deviation, ackermann_ratio_values; signed = signed, lower_default = 30.0)

    GLMakie.lines!(ax_roll_camber, roll_values, left_camber; color = :royalblue, linewidth = 3)
    GLMakie.lines!(ax_roll_camber, roll_values, right_camber; color = :darkorange, linewidth = 3)
    GLMakie.lines!(ax_roll_wheel_angle, roll_values, left_wheel_angle; color = :royalblue, linewidth = 3)
    GLMakie.lines!(ax_roll_wheel_angle, roll_values, right_wheel_angle; color = :darkorange, linewidth = 3)
    GLMakie.lines!(ax_roll_track_width, roll_values, track_width; color = :seagreen, linewidth = 3)
    GLMakie.lines!(ax_roll_ackermann_deviation, roll_values, ackermann_ratio_values; color = :firebrick, linewidth = 3)

    return fig
end
