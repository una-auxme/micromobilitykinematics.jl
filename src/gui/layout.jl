
#######################| Naming
#
# Layout Wrapper  |	layout_*_section  |	   l_*s  |	  z.B. lps, lpls, lcs
#
"""
    interactionlyt(θ_max, chassis, steering, suspension)

Constructs the full interactive GUI layout, integrating all major UI sections into a single figure.

# Arguments
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining maximum steering angles.
- `chassis`: The `Chassis` object representing the vehicle frame.
- `steering`: The `Steering` system with kinematic configuration.
- `suspension`: The `Suspension` system influencing geometry and steering behavior.

# Description
This function assembles a complete GUI figure (`GLMakie.Figure`) with the following layout structure:
- `section_plot`: All visual plots (geometry, radii, Ackermann ratio) in rows 1–2, columns 1–3.
- `section_angle`: Slider controls for steering angles in row 3, column 1.
- `section_param`: Parameter configuration (e.g., rod lengths) in row 3, column 2.
- `section_damper`: Damper settings (left/right compression) in row 3, column 3.
- `section_plot_settings`: Menu and buttons to select plot mode, reset or save in row 4, column 1.
- `section_info`: Display of computed values (objective, ratio, etc.) in row 4, column 3.

It also sets relative column widths to evenly distribute the interface.

# Returns
- `InteractionLyt`: A structured container with references to all major UI sections and the main figure.
"""
function interactionlyt(θ_max, chassis, steering, suspension)

    interaction_lyt = InteractionLyt()

    # Figure
    interaction_lyt.fig = GLMakie.Figure(size = (1400, 1200))


    interaction_lyt.section_plot = layout_section_plot(interaction_lyt.fig,(1:2,1:3), θ_max, chassis, steering, suspension)
    interaction_lyt.section_angle = layout_section_angles(interaction_lyt.fig,(3,1),θ_max)
    interaction_lyt.section_param = layout_section_param(interaction_lyt.fig,(3,2), steering)
    interaction_lyt.section_damper = layout_section_damper(interaction_lyt.fig,(3,3))
    interaction_lyt.section_plot_settings = layout_section_plot_settings(interaction_lyt.fig,(4,1)) 
    interaction_lyt.section_info = layout_section_info(interaction_lyt.fig,(4,3))

    colsize!(interaction_lyt.fig.layout, 1, Relative(0.3))  # Spalte 1 = 30%
    colsize!(interaction_lyt.fig.layout, 2, Relative(0.3))  # Spalte 2 = 30%
    colsize!(interaction_lyt.fig.layout, 3, Relative(0.3))  # Spalte 3 = 30%

    return interaction_lyt

end


"""
    layout_section_angles(fig, slot, θ_max)



Creates a UI section with sliders for configuring steering rotation angles (θx, θy, θz).

# Arguments
- `fig`: A `GLMakie.Figure` object where the slider section will be placed.
- `slot`: A tuple `(row, col)` specifying the position in the figure's grid layout.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining the maximum values for the respective rotation angles.

# Description
This function:
- Initializes a new `AngleSection` containing:
  - A titled label ("Rotation angle configuration")
  - A `SliderGrid` for the three angles `θx`, `θy`, and `θz`, each with a range from 0 to their respective max value.
- Embeds the slider layout into the specified slot of the provided figure.
- Applies a vertical compression using `rowgap!` for tighter layout appearance.

The returned `AngleSection` struct contains references to the sliders and layout for further UI integration or event binding.

# Returns
- `AngleSection`: A structured object containing the slider layout, title label, and angle values.
"""
function layout_section_angles(fig, slot, θ_max) 

    angle_section = AngleSection()

    θx_max , θy_max, θz_max  = θ_max

    angle_section.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    angle_section.slot = slot #slot = (3,1)

    fig[row,col] = angle_section.lyt

    angle_section.title = Label(angle_section.lyt[1,1], "Rotation angle configuration", fontsize = 15)


    angle_section.sg_θ = SliderGrid( angle_section.lyt[2, 1],
                                    (label = "θx", range = 0.0:1.0:θx_max, format = "{:.1f}°", startvalue = 0),
                                    (label = "θy", range = 0.0:0.1:θy_max, format = "{:.1f}°", startvalue = 0),
                                    (label = "θz", range = 0.0:1.0:θz_max, format = "{:.1f}°", startvalue = 0),
                                    width = 350,
                                    tellheight = false)

    rowgap!(angle_section.lyt, 1, -100)

    return angle_section
    
end

"""
    layout_section_param(fig,slot)


Creates a UI section with sliders for configuring various component parameters.

# Arguments
- `fig`: A `GLMakie.Figure` object where the slider section will be placed.
- `slot`: A tuple `(row, col)` specifying the position in the figure's grid layout.

# Description
This function initializes a `ParamSection` UI element consisting of:
- A title label: "Components configuration"
- A `SliderGrid` with four sliders for key vehicle parameters:
  - `θx radius`: Range 50–100 mm
  - `θz radius`: Range 50–200 mm
  - `track lever`: Range 70–200 mm
  - `tie rod`: Range 195–260 mm

Each slider uses a consistent formatting style and layout width, and the section is compressed vertically using `rowgap!` for compactness.

# Returns
- `ParamSection`: A structured object containing the parameter slider layout, title, and values.
"""
function layout_section_param(fig,slot, steering)

    θx_radius = steering.init_steering.θx_radius
    θz_radius = steering.init_steering.θz_radius
    track_lever_length = steering.init_steering.track_lever_length
    tie_rod_length = steering.init_steering.tie_rod_length


    section_param = ParamSection()

    section_param.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    section_param.slot = slot #slot = (3,2)

    fig[row,col] = section_param.lyt

    section_param.title = Label(section_param.lyt[1,1], "Components configuration", fontsize = 15)

    section_param.sg_param = SliderGrid(section_param.lyt[2, 1],
                                        (label = "θx radius", range = 50:1:100, format = "{:.1f}mm", startvalue = θx_radius),
                                        (label = "θz radius", range = 50:1:200, format = "{:.1f}mm", startvalue = θz_radius),
                                        (label = "track lever", range = 70:1:200, format = "{:.1f}mm", startvalue = track_lever_length),
                                        (label = "tie rod", range = 195:1:260, format = "{:.1f}mm", startvalue = tie_rod_length),
                                        width = 350,
                                        tellheight = false)
    rowgap!(section_param.lyt, 1, -50)

    return section_param
end

"""
    layout_section_plot_settings(fig,slot)

Creates a UI section for plot settings, including plot mode selection and save/reset controls.

# Arguments
- `fig`: A `GLMakie.Figure` into which this section will be inserted.
- `slot`: A tuple `(row, col)` specifying the layout position within the figure.

# Description
This function builds a `PlotSettingsSection` layout which includes:
- A title label: "Plot settings"
- A dropdown `Menu` to select the plot mode (options: `"geometry"`, `"radii"`, `"ackermannratio"`, `"ackermannratio surface plot"`)
- A button to reset all relevant UI settings (`"Reset"`)
- A button to save the currently displayed plot (`"Save current plot"`)
- A button to save all available plots at once (`"Save all plots"`)

All elements are arranged within a sub-layout (`suplyt`) embedded in the main layout cell. Layout compression is applied to keep the section compact.

# Returns
- `PlotSettingsSection`: An object containing references to the menu and control buttons for plot interactions.
"""
function layout_section_plot_settings(fig,slot)

    section_plot_settings = PlotSettingsSection()

    section_plot_settings.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    section_plot_settings.slot = slot #slot = (4,1)

    fig[row, col] = section_plot_settings.lyt  # Sub-Layout einfügen

    section_plot_settings.title = Label(section_plot_settings.lyt[1,1], "Plot settings", fontsize = 15)

    # 
    
    section_plot_settings.suplyt = GridLayout(tellheight = false)
    section_plot_settings.lyt[2,1] = section_plot_settings.suplyt

    
    section_plot_settings.menu = Menu(section_plot_settings.suplyt[1, 1], options = ["geometry", "radii", "ackermannratio", "ackermannratio surface plot"], default = "geometry",width = 300)
    section_plot_settings.btn_reset = Button(section_plot_settings.suplyt[2, 1], label = "Reset",width = 300)
    section_plot_settings.btn_save = Button(section_plot_settings.suplyt[3, 1], label = "Save current plot",width = 300)
    section_plot_settings.btn_save_all = Button(section_plot_settings.suplyt[4, 1], label = "Save all plots",width = 300)

    rowgap!(section_plot_settings.lyt, 1, 0)

    return section_plot_settings

end


"""
    layout_section_damper(fig,slot)


Creates a UI section for adjusting damper (shock absorber) compression settings.

# Arguments
- `fig`: A `GLMakie.Figure` where this section will be placed.
- `slot`: A tuple `(row, col)` specifying the grid layout position in the figure.

# Description
This function sets up a `DamperSection` consisting of:
- A title label: "Dumper settings"
- A `SliderGrid` with two sliders for:
  - `compression left`: Adjustable from 0% to 100%, with default at 30%
  - `compression right`: Adjustable from 0% to 100%, with default at 30%

The layout is compressed vertically using `rowgap!` for a cleaner UI appearance.

This section is intended to let users modify the damper configuration interactively, which can then be used in simulation or geometry computations.

# Returns
- `DamperSection`: A structured object holding the layout and compression slider values.
"""
function layout_section_damper(fig,slot)

    section_damper = DamperSection()

    section_damper.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    section_damper.slot = slot #slot = (3,3)

    fig[row,col] = section_damper.lyt

    section_damper.title = Label(section_damper.lyt[1,1], "Dumper settings", fontsize = 15)


    section_damper.sg_compr = SliderGrid(section_damper.lyt[2, 1],
                                (label = "compression left", range = 0:1:100, format = "{:.1f}%", startvalue = 30),
                                (label = "compression right", range = 0:1:100, format = "{:.1f}%", startvalue = 30),
                                width = 350,
                                tellheight = false)

    rowgap!(section_damper.lyt, 1, -100)

    return section_damper

end


"""
    layout_section_info(fig,slot)

Creates a UI section that displays computed simulation results and steering metrics.

# Arguments
- `fig`: A `GLMakie.Figure` where this info section will be placed.
- `slot`: A tuple `(row, col)` specifying the position in the figure's layout grid.

# Description
This function sets up an `InfoSection` composed of:
- A title label: "Information"
- A sub-layout (`suplyt`) containing several `Textbox` elements for displaying:
  - `objective`: The optimization or alignment objective value.
  - `ackermannratio`: The Ackermann steering ratio (in %).
  - `δi`: Inner wheel steering angle (in degrees).
  - `δo`: Outer wheel steering angle (in degrees).
  - `radius`: Turning radius (in mm).

Textboxes are initially populated with placeholders and are meant to be updated dynamically during simulation or user interaction. The layout is arranged in two columns and compacted vertically for a clean appearance.

# Returns
- `InfoSection`: A structured object containing references to the display elements for dynamic updates.
"""
function layout_section_info(fig,slot)

    section_info = InfoSection()

    section_info.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    section_info.slot = slot #slot = (4,3)

    fig[row, col] = section_info.lyt  # Sub-Layout einfügen

    section_info.title = Label(section_info.lyt[1,1], "Information", fontsize = 15)

    # Füge mehrere Buttons in das verschachtelte Layout ein:

    section_info.suplyt = GridLayout(tellheight = false)
    section_info.lyt[2,1] = section_info.suplyt

    section_info.tb_obj = Textbox(section_info.suplyt[1, 1:2], placeholder = "objective: ",width = 300)
    section_info.tb_ratio = Textbox(section_info.suplyt[2, 1:2], placeholder = "ackermannratio: ",width = 300)
    section_info.tb_δi = Textbox(section_info.suplyt[3, 1], placeholder = "δi: ",width = 150)
    section_info.tb_δo = Textbox(section_info.suplyt[4, 1], placeholder = "δo: ",width = 150)
    section_info.tb_rad = Textbox(section_info.suplyt[3, 2], placeholder = "radius: ",width = 150)

    rowgap!(section_info.lyt, 1, 0)

    return section_info

end



"""
    layout_section_plot(fig,slot, θ_max, chassis, steering, suspension)


Initializes and returns a `PlotSection` containing all core visualizations related to the steering system.

# Arguments
- `fig`: A `GLMakie.Figure` into which the plot section will be inserted.
- `slot`: A tuple `(row, col)` defining the layout position in the figure's grid.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` indicating the maximum steering angles.
- `chassis`: The `Chassis` model representing the vehicle frame.
- `steering`: The `Steering` object containing all kinematic and joint configuration data.
- `suspension`: The `Suspension` model affecting wheel positions and geometry.

# Description
This function constructs a complete set of plots related to the vehicle's steering dynamics by:
- Rendering a **3D geometry plot** of the steering linkage (`geom_plot!`)
- Drawing a **2D plot** of the turning radii over θz (`radii_plot!`)
- Creating a **2D Ackermann ratio plot** with dynamic min/max indicators (`ratio_plot!`)
- Adding a **3D surface plot** of the Ackermann ratio over θx and θz (`ratio_surface_plot!`)

Each plot is added to the same layout cell in the figure but is initially hidden. Visibility can be controlled interactively via the plot settings menu.

# Returns
- `PlotSection`: A container object holding references to all plot axes and their reactive observables.
"""
function layout_section_plot(fig,slot, θ_max, chassis, steering, suspension)
    θx_max, θy_max, θz_max = θ_max

    section_plot = PlotSection()

    row = slot[1]
    col = slot[2]
    section_plot.slot = slot #slot = (4,3)

    #######################################################| Layout 
    #####################| Layout 3D Axis
    ############| Geometry Plot

    geom_plot!(fig,section_plot,steering)


    #############| Radii Plot

    radii_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


    ##############| ackermannratio Plot
    ratio_plot!(fig,section_plot, θ_max, chassis, steering, suspension)

    ratio_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)

    return section_plot
end 

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
function geom_plot!(fig,section_plot, steering)

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



    ############| Geometry Observervar                
    section_plot.obs_rotation = Observable(rotational_coponent)
    section_plot.obs_geom_left = Observable(left_steering_connections)
    section_plot.obs_geom_right = Observable(right_steering_connections)
    section_plot.obs_stationary = Observable(stationary)



    ############| Geometry Ploting   
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_rotation, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_geom_left, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_geom_right, markersize=10)
    GLMakie.scatter!(section_plot.ax_geom, section_plot.obs_stationary, markersize=10)

    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_rotation)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_geom_left)
    GLMakie.lines!(section_plot.ax_geom, section_plot.obs_geom_right)

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
    radii_θz = steering_radii_θz(1.0,0.0,θz_max,chassis, steering, suspension)

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

    ratio = ackermannratio_θz(0.0,0.0,θz_max,chassis, steering, suspension)

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
                                                text = "Min → $(round(section_plot.obs_ratio_max[], digits=2))%",
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

    ratio_surface = ackermannratio_surface(chassis, steering, suspension, θ_max)


    ############| Ackermannratio Observervar 
    section_plot.obs_ratio_surface = Observable(ratio_surface)

    ############| Ackermannratio Ploting  

    GLMakie.surface!(section_plot.ax_ratio_surface , 0.0:1.0:θx_max, 0.0:1.0:θz_max, section_plot.obs_ratio_surface; colormap = :darkterrain)
    
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
    steeringkinematicsMOVED!(θ, steering, suspension)
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

    section_plot.obs_rotation[] = rotational_coponent
    section_plot.obs_geom_left[] = left_steering_connections
    section_plot.obs_geom_right[] = right_steering_connections
    section_plot.obs_stationary[] = stationary

end 
