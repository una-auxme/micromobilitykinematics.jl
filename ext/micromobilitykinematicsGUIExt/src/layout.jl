
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
    interaction_lyt.fig = GLMakie.Figure(size = (1450, 1200))

    interaction_lyt.section_plot = layout_section_plot(interaction_lyt.fig,(1:2,1:3), θ_max, deepcopy(chassis), deepcopy(steering), deepcopy(suspension))
    interaction_lyt.section_angle = layout_section_angles(interaction_lyt.fig,(3,1),θ_max, deepcopy(steering))
    interaction_lyt.section_param = layout_section_param(interaction_lyt.fig,(3,2), deepcopy(steering))
    interaction_lyt.section_damper = layout_section_damper(interaction_lyt.fig,(3,3))
    interaction_lyt.section_plot_settings = layout_section_plot_settings(interaction_lyt.fig,(4,1)) 
    interaction_lyt.section_info = layout_section_info(interaction_lyt.fig,(4,3))
    interaction_lyt.section_error =  layout_section_error(interaction_lyt.fig,(4,2))

    colsize!(interaction_lyt.fig.layout, 1, Relative(0.3))  # Spalte 1 = 30%
    colsize!(interaction_lyt.fig.layout, 2, Relative(0.3))  # Spalte 2 = 30%
    colsize!(interaction_lyt.fig.layout, 3, Relative(0.3))  # Spalte 3 = 30%

    rowgap!(interaction_lyt.fig.layout, 3, -80)

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
function layout_section_angles(fig, slot, θ_max, steering) 

    angle_section = AngleSection()

    θx_max , θy_max, θz_max  = θ_max
    θx = steering.θx 
    θy = steering.θy 
    θz = steering.θz 

    angle_section.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    angle_section.slot = slot #slot = (3,1)

    fig[row,col] = angle_section.lyt

    angle_section.title = Label(angle_section.lyt[1,1], "Rotation angle configuration", fontsize = 15)


    angle_section.sg_θ = SliderGrid( angle_section.lyt[2, 1],
                                    (label = "θx", range = 0.0:1.0:θx_max, format = "{:.1f}°", startvalue = θx),
                                    (label = "θy", range = 0.0:0.1:θy_max, format = "{:.1f}°", startvalue = θy),
                                    (label = "θz", range = 0.0:1.0:θz_max, format = "{:.1f}°", startvalue = θz),
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

    
    section_plot_settings.menu = Menu(section_plot_settings.suplyt[1, 1], 
                                        options = ["geometry", 
                                                    "radii", 
                                                    "ackermannratio", 
                                                    "ackermannratio surface plot", 
                                                    "steering vs wheel angles", 
                                                    "ackermann deviation", 
                                                    "ackermann deviation surface",
                                                    "compression vs wheel angles"], 
                                        default = "geometry", 
                                        width = 300)
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
    section_info.tb_δo = Textbox(section_info.suplyt[3, 2], placeholder = "δo: ",width = 150)
    section_info.tb_rad = Textbox(section_info.suplyt[4, 1], placeholder = "radius: ",width = 150)
    section_info.tb_min_rad = Textbox(section_info.suplyt[4, 2], placeholder = "min. radius: ",width = 150)
    


    rowgap!(section_info.lyt, 1, 0)

    return section_info

end


function layout_section_error(fig,slot)

    section_error = ErrorSection()

    section_error.lyt = GridLayout(tellheight = false)

    row = slot[1]
    col = slot[2]
    section_error.slot = slot #slot = (4,3)

    fig[row, col] = section_error.lyt  # Sub-Layout einfügen

    section_error.title = Label(section_error.lyt[1,1], "Error Infomation", fontsize = 15)

    # Füge mehrere Buttons in das verschachtelte Layout ein:

    section_error.suplyt = GridLayout(tellheight = false)
    section_error.lyt[2,1] = section_error.suplyt

    section_error.tb_error_id = Textbox(section_error.suplyt[1, 1], placeholder = "Error ID: ", width = 300)
    section_error.tb_error_type = Textbox(section_error.suplyt[2, 1], placeholder = "Error Type: ", width = 300)
    section_error.tb_error_msg = Label(section_error.suplyt[3, 1], text = "Error Msg.: ", width = 350, tellheight = true, tellwidth = false, word_wrap = true)


    #rowsize!(fig.layout, row,  Fixed(3))      # Zeile 5 kollabiert
    #rowgap!(fig.layout, row - 1, 0)          # Gap zwischen 4 und 5 = 0

    rowgap!(section_error.lyt, 1, 0)

    return section_error

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

    geom_plot!(fig,section_plot,steering, suspension)


    #############| Radii Plot

    radii_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


    ##############| ackermannratio Plot
    ratio_plot!(fig, section_plot, θ_max, chassis, steering, suspension)

    ratio_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


    ##############| θ_vs_δ Plot
    θ_vs_δ_plot!(fig, section_plot, θ_max, steering, suspension) 


    ##############| ackermann devitation Plot
    deviation_plot!(fig, section_plot, θ_max, chassis, steering, suspension)

    deviation_surface_plot!(fig,section_plot, θ_max, chassis, steering, suspension)


    compr_vs_δ_plot!(fig, section_plot, steering, suspension)

    return section_plot
end 
