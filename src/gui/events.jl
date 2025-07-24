

"""
    event_slider_θx(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    tiggers slider event for θx

Registers and handles the slider event for the θx steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the θx slider (the first slider in the `section_angle.sg_θ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(θx, θy, θz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"geometry"`, `"radii"`, or `"ackermannratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_θx(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)
    
    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info




    on(section_angle.sg_θ.sliders[1].value) do val
        θx = val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end

"""
    event_slider_θy(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers and handles the slider event for the θy steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the θy slider (the second slider in the `section_angle.sg_θ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(θx, θy, θz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"geometry"`, `"radii"`, or `"ackermannratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_θy(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_angle.sg_θ.sliders[2].value) do val
            θx = section_angle.sg_θ.sliders[1].value.val
            θy = val
            θz = section_angle.sg_θ.sliders[3].value.val

            # Calculation
            update_geometry!((θx,θy,θz),section_plot,steering, suspension)
            obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
            ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)


            # 

            if section_plot_settings.menu.selection.val == "geometry"
                section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"      
            end

            if section_plot_settings.menu.selection.val == "radii"
                section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
                section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "ackermannratio"
                section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
                ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
                section_plot.obs_ratio_θz[] = ratio_θz
                section_plot.obs_ratio_min[] = minimum(ratio_θz)
                section_plot.obs_ratio_max[] = maximum(ratio_θz)
            end


            if section_plot_settings.menu.selection.val == "steering vs wheel angles"
                section_plot.ax_θ_vs_δ_surface.title = "steering vs wheel angles for (θx, θy, θz) = ($θx_max,$θy,$θz_max)"
                section_plot.ax_θ_vs_δ_surface.title = "steering vs wheel angles for (θx, θy, θz) = ($θx_max,$θy,$θz_max)"
                section_plot.obs_θ_vs_δi_surface[] = ax_θ_vs_δi(steering, (θx_max, θy, θz_max))
                section_plot.obs_θ_vs_δo_surface[] = ax_θ_vs_δo(steering, (θx_max, θy, θz_max))
            end


            # Updating
            section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
        end
end


"""
    event_slider_θz(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers and handles the slider event for the θz steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the θz slider (the third slider in the `section_angle.sg_θ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(θx, θy, θz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"geometry"`, `"radii"`, or `"ackermannratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_θz(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    
    on(section_angle.sg_θ.sliders[3].value) do val
        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = val

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        #

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"      
        end

        # Updating 
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end

"""
    event_slider_θ(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


Initializes all θ angle slider event handlers (θx, θy, θz) in the user interface.

# Arguments
- `args...`: Variadic arguments passed to each of the individual event handler functions:
  - `interaction_lyt::InteractionLyt`
  - `θ_max`: Tuple of maximum angle values `(θx_max, θy_max, θz_max)`
  - `chassis::Chassis`
  - `steering::Steering`
  - `suspension::Suspension`

# Description
This convenience function initializes the event listeners for all three rotation angle sliders:
- `event_slider_θx` for pitch/roll angle,
- `event_slider_θy` for yaw angle,
- `event_slider_θz` for roll/pitch angle.

Each handler updates the vehicle model, calculations, and corresponding plots when the slider value changes.

# Returns
Nothing. Registers the slider event callbacks via side effects.
"""
function event_slider_θ(args...)
    event_slider_θx(args...)
    event_slider_θy(args...)
    event_slider_θz(args...)
end


"""
    event_slider_left_compression(interaction_lyt::InteractionLyt,
                                    θ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)

Initializes the event handler for the left damper compression slider in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: Layout container holding all relevant UI components.
- `θ_max`: Tuple of maximum angle values `(θx_max, θy_max, θz_max)` used for plotting limits and calculations.
- `chassis::Chassis`: Vehicle chassis representation.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension system model, including damper compression states.

# Description
This function registers an event listener for the left suspension damper compression slider. When the user changes the slider value, the following occurs:

- The new compression value is applied to the suspension model.
- The current orientation angles `(θx, θy, θz)` are read from the interface.
- Vehicle geometry is recalculated based on updated suspension and steering inputs.
- Various plots are updated depending on the selected plot mode:
  - Geometry plots (`"geometry"`)
  - Turning radius plots (`"radii"`)
  - Ackermann ratio plots (`"ackermannratio"`)
- Key information (objective value, Ackermann ratio, steering angles, turning radius) is displayed and refreshed in the interface.

# Returns
Nothing. Registers the event handler as a side effect.
"""
function event_slider_left_compression(interaction_lyt::InteractionLyt,
                                        θ_max, 
                                        chassis::Chassis, 
                                        steering::Steering,
                                        suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info

    on(section_damper.sg_compr.sliders[1].value) do val

        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        left_compr = val
        right_compr = section_damper.sg_compr.sliders[2].value.val

        suspension.damper[1].compression = left_compr
        suspension.damper[2].compression = right_compr


        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end

        if section_plot_settings.menu.selection.val == "steering vs wheel angles"
            section_plot.ax_θ_vs_δ_surface.title = "steering vs wheel angles for (θx, θy, θz) = ($θx_max,$θy,$θz_max)"
            section_plot.obs_θ_vs_δi_surface[] = ax_θ_vs_δi(steering, (θx_max, θy, θz_max))
            section_plot.obs_θ_vs_δo_surface[] = ax_θ_vs_δo(steering, (θx_max, θy, θz_max))
        end

        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end
end


"""
    event_slider_right_compression(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

Initializes the event handler for the right damper compression slider in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: Layout container holding all relevant UI components.
- `θ_max`: Tuple of maximum angle values `(θx_max, θy_max, θz_max)` used for plotting limits and calculations.
- `chassis::Chassis`: Vehicle chassis representation.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension system model, including damper compression states.

# Description
This function registers an event listener for the right suspension damper compression slider. When the user changes the slider value, the following occurs:

- The new compression value is applied to the suspension model.
- The current orientation angles `(θx, θy, θz)` are read from the interface.
- Vehicle geometry is recalculated based on updated suspension and steering inputs.
- Various plots are updated depending on the selected plot mode:
  - Geometry plots (`"geometry"`)
  - Turning radius plots (`"radii"`)
  - Ackermann ratio plots (`"ackermannratio"`)
- Key information (objective value, Ackermann ratio, steering angles, turning radius) is displayed and refreshed in the interface.

# Returns
Nothing. Registers the event handler as a side effect.
"""
function event_slider_right_compression(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info

    on(section_damper.sg_compr.sliders[2].value) do val

        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        left_compr = section_damper.sg_compr.sliders[1].value.val
        right_compr = val

        suspension.damper[1].compression = left_compr
        suspension.damper[2].compression = right_compr

        println(right_compr)

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end

        if section_plot_settings.menu.selection.val == "steering vs wheel angles"
            section_plot.ax_θ_vs_δ_surface.title = "steering vs wheel angles for (θx, θy, θz) = ($θx_max,$θy,$θz_max)"
            section_plot.obs_θ_vs_δi_surface[] = ax_θ_vs_δi(steering, (θx_max, θy, θz_max))
            section_plot.obs_θ_vs_δo_surface[] = ax_θ_vs_δo(steering, (θx_max, θy, θz_max))
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end
end


"""
    event_slider_compression(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

Initializes both left and right damper compression slider event handlers in the user interface.

# Arguments
- `args...`: Variadic arguments passed to the respective handler functions:
  - `interaction_lyt::InteractionLyt`: UI layout container.
  - `θ_max`: Tuple of maximum rotation angles `(θx_max, θy_max, θz_max)`.
  - `chassis::Chassis`: Vehicle chassis representation.
  - `steering::Steering`: Steering system model.
  - `suspension::Suspension`: Suspension system model.

# Description
This convenience function initializes the event listeners for both damper compression sliders:
- `event_slider_left_compression` for the left damper,
- `event_slider_right_compression` for the right damper.

Each slider updates the suspension model and triggers recalculations and plot updates based on the new compression values and current orientation.

# Returns
Nothing. Registers the slider event callbacks via side effects.
"""
function event_slider_compression(args...)
    event_slider_left_compression(args...) 
    event_slider_right_compression(args...) 
end

"""
    event_menu_plot_settings(interaction_lyt::InteractionLyt,
                                    θ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)

Registers an event listener for the plot settings menu and updates plot visibility,
sliders, and computed values accordingly.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object containing UI elements such as plots, sliders, and menus.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` specifying the maximum values for the steering angles.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system model.
- `suspension::Suspension`: The suspension system model.

# Description
This function handles changes to the selection in the plot settings menu. Depending on the selected mode (`"geometry"`, `"radii"`, `"ackermannratio"`, or `"ackermannratio surface plot"`), it:
- Shows or hides the appropriate plot axes.
- Toggles the visibility of angle sliders (θx, θy, θz) accordingly.
- Computes and updates the relevant observables (`obs_radii_θz`, `obs_ratio_θz`, `obs_ratio_surface`) used for plotting.
- Adjusts the display based on current slider values and system state.

# Returns
Nothing. Updates the UI and observable values as a side effect of menu interactions.
"""
function event_menu_plot_settings(interaction_lyt::InteractionLyt,
                                    θ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)



    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info

    on(section_plot_settings.menu.selection) do sel

        if sel == "geometry"
            section_plot.ax_geom.blockscene.visible[] = true 
            section_plot.ax_radii.blockscene.visible[] = false
            section_plot.ax_ratio.blockscene.visible[] = false
            section_plot.ax_ratio_surface.blockscene.visible[] = false
            section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false

            # sliders
            section_angle.sg_θ.sliders[1].blockscene.visible[] = true
            section_angle.sg_θ.sliders[2].blockscene.visible[] = true
            section_angle.sg_θ.sliders[3].blockscene.visible[] = true


        end


        if sel == "radii"
            section_plot.ax_geom.blockscene.visible[] = false
            section_plot.ax_radii.blockscene.visible[] = true
            section_plot.ax_ratio.blockscene.visible[] = false
            section_plot.ax_ratio_surface.blockscene.visible[] = false
            section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false

            # sliders
            section_angle.sg_θ.sliders[1].blockscene.visible[] = true
            section_angle.sg_θ.sliders[2].blockscene.visible[] = true
            section_angle.sg_θ.sliders[3].blockscene.visible[] = false


            θx = section_angle.sg_θ.sliders[1].value.val
            θy = section_angle.sg_θ.sliders[2].value.val

            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end


        if sel == "ackermannratio"
            section_plot.ax_geom.blockscene.visible[] = false
            section_plot.ax_radii.blockscene.visible[] = false
            section_plot.ax_ratio.blockscene.visible[] = true
            section_plot.ax_ratio_surface.blockscene.visible[] = false
            section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false

            # sliders
            section_angle.sg_θ.sliders[1].blockscene.visible[] = true
            section_angle.sg_θ.sliders[2].blockscene.visible[] = true
            section_angle.sg_θ.sliders[3].blockscene.visible[] = false


            θx = section_angle.sg_θ.sliders[1].value.val
            θy = section_angle.sg_θ.sliders[2].value.val

            section_plot.obs_ratio_θz[] = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if sel == "ackermannratio surface plot"
            section_plot.ax_geom.blockscene.visible[] = false
            section_plot.ax_radii.blockscene.visible[] = false
            section_plot.ax_ratio.blockscene.visible[] = false
            section_plot.ax_ratio_surface.blockscene.visible[] = true
            section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = false
            # sliders
            section_angle.sg_θ.sliders[1].blockscene.visible[] = false
            section_angle.sg_θ.sliders[2].blockscene.visible[] = true
            section_angle.sg_θ.sliders[3].blockscene.visible[] = false

            θy = section_angle.sg_θ.sliders[2].value.val

            section_plot.obs_ratio_surface[] = ackermannratio_surface(chassis, steering, suspension, (θx_max,θy,θz_max))


        end

        if sel =="steering vs wheel angles"
            section_plot.ax_geom.blockscene.visible[] = false
            section_plot.ax_radii.blockscene.visible[] = false
            section_plot.ax_ratio.blockscene.visible[] = false
            section_plot.ax_ratio_surface.blockscene.visible[] = false
            section_plot.ax_θ_vs_δ_surface.blockscene.visible[] = true
            # sliders
            section_angle.sg_θ.sliders[1].blockscene.visible[] = false
            section_angle.sg_θ.sliders[2].blockscene.visible[] = true
            section_angle.sg_θ.sliders[3].blockscene.visible[] = false

            θy = section_angle.sg_θ.sliders[2].value.val

            section_plot.ax_θ_vs_δ_surface.title = "steering vs wheel angles for (θx, θy, θz) = ($θx_max,$θy,$θz_max)"
            section_plot.obs_θ_vs_δi_surface[] = ax_θ_vs_δi(steering, (θx_max, θy, θz_max))
            section_plot.obs_θ_vs_δo_surface[] = ax_θ_vs_δo(steering, (θx_max, θy, θz_max))
        end

    end
end



"""
    event_btn_save(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers a callback for the 'Save' button to export the currently selected plot view
based on the active steering angles.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing the user interface, including plots, controls, and buttons.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` representing the maximum allowable values for the steering angles.
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The vehicle's steering system.
- `suspension::Suspension`: The vehicle's suspension system.

# Description
When the save button is clicked, this function:
- Reads the current values of the steering angles `(θx, θy, θz)` from the sliders.
- Determines which plot type is currently selected via the menu.
- Generates the appropriate plot (geometry, Ackermann ratio over θz, or surface plot).
- Saves the figure as a PNG file using a filename that includes the angle values (where applicable).
- Displays the original figure again after saving.

Currently, no file is saved if the `"radii"` plot type is selected.

# Returns
Nothing. The function performs file-saving and UI updates as side effects of the button click event.
"""
function event_btn_save(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info

    on(section_plot_settings.btn_save.clicks) do n

        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val

        θ = (θx, θy, θz)

        if section_plot_settings.menu.selection.val == "geometry"
            fig_geo = geometry_plot(θ, steering, suspension)
            GLMakie.save("geometry(θx,θy,θz)=($θx,$θy,$θz).png",fig_geo)
            GLMakie.display(fig)

        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            fig_ratio = ackermannratio_θz_plot(θx,θy,θz_max, chassis, steering, suspension)
            GLMakie.save("ackermannratio(θx,θy,θz)=($θx,$θy,θz).png",fig_ratio)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "radii"

        end

        if section_plot_settings.menu.selection.val == "ackermannratio surface plot"
            fig_ratio = ackermannratio_sufrace_plot(chassis, steering, suspension,(θx_max,θy,θz_max))
            GLMakie.save("ackermannratio_surface_plot.png",fig_ratio)
            GLMakie.display(fig)
        end

    end
end



"""
    event_btn_save_all(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers a callback for the 'Save All' button to export all major plot views
(geometry, Ackermann ratio over θz, and Ackermann ratio surface plot)
based on the current steering angle configuration.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout structure containing the full UI with figures, sliders, and controls.
- `θ_max`: A tuple `(θx_max, θy_max, θz_max)` defining the upper bounds for steering angles.
- `chassis::Chassis`: The vehicle's chassis system.
- `steering::Steering`: The steering system configuration.
- `suspension::Suspension`: The suspension model of the vehicle.

# Description
On button click, this function:
- Reads the current values of the steering angles `(θx, θy, θz)` from the UI sliders.
- Generates the following plots:
  - Steering geometry plot for `(θx, θy, θz)`
  - Ackermann ratio plot over varying `θz`
  - Ackermann ratio surface plot over the full angle space
- Saves each figure to disk with an appropriate filename.
- Ensures the original figure remains displayed after saving.

This is a convenience function to quickly export all relevant analysis visualizations in a single action.

# Returns
Nothing. Executes saving and visualization as side effects triggered by the button click event.
"""
function event_btn_save_all(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_plot_settings.btn_save_all.clicks) do n

        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val

        θ = (θx, θy, θz)


        fig_geom = geometry_plot(θ, steering, suspension)
        GLMakie.save("geometry(θx,θy,θz)=($θx,$θy,$θz).png",fig_geom)
        GLMakie.display(fig)


        fig_ratio = ackermannratio_θz_plot(θx,θy,θz_max, chassis, steering, suspension)
        GLMakie.save("ackermannratio(θx,θy,θz)=($θx,$θy,θz).png",fig_ratio)
        GLMakie.display(fig)


        fig_ratio = ackermannratio_sufrace_plot(chassis, steering, suspension,(θx_max,θy,θz_max))
        GLMakie.save("ackermannratio_surface_plot.png",fig_ratio)
        GLMakie.display(fig)

    end

    GLMakie.display(fig)
end



"""
    event_btn_reset(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Initializes the reset button event handler in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: UI layout container with all relevant interface elements.
- `θ_max`: Tuple of maximum rotation angles `(θx_max, θy_max, θz_max)` used in geometry/radius/ratio calculations.
- `chassis::Chassis`: Representation of the vehicle chassis.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension model, including damper states.

# Description
This function registers an event listener for the reset button in the UI. When triggered, the handler:

- Resets all rotation angles `(θx, θy, θz)` to zero.
- Resets both left and right damper compression values to a default (30.0 mm).
- Updates the geometry, objective value, Ackermann ratio, and turning radius.
- Refreshes the UI plots and updates all related information displays.
- Ensures that the visual output corresponds to the neutral vehicle state.

# Returns
Nothing. Registers the reset button callback and triggers updates via side effects.
"""
function event_btn_reset(interaction_lyt::InteractionLyt,
                            θ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_plot_settings.btn_reset.clicks) do n
        θx, θy, θz = θ = (0, 0, 0)


        suspension.damper[1].compression = 30.0
        suspension.damper[2].compression = 30.0


        set_close_to!(section_angle.sg_θ.sliders[1], 0.0)
        set_close_to!(section_angle.sg_θ.sliders[2], 0.0)
        set_close_to!(section_angle.sg_θ.sliders[3], 0.0)

        set_close_to!(section_damper.sg_compr.sliders[1], 30.0)
        set_close_to!(section_damper.sg_compr.sliders[2], 30.0)

        set_close_to!(section_param.sg_param.sliders[1], steering.init_steering.θx_radius)
        set_close_to!(section_param.sg_param.sliders[2], steering.init_steering.θz_radius)
        set_close_to!(section_param.sg_param.sliders[3], steering.init_steering.track_lever_length)
        set_close_to!(section_param.sg_param.sliders[4], steering.init_steering.tie_rod_length)


        @show steering.rotational_component.x_rotational_radius = steering.init_steering.θx_radius
        @show steering.rotational_component.z_rotational_radius = steering.init_steering.θz_radius
        @show steering.track_lever.length = steering.init_steering.track_lever_length
        @show steering.tie_rod.length = steering.init_steering.tie_rod_length



        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

    GLMakie.display(fig)
end


function event_slider_param_θx_radius(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_param.sg_param.sliders[1].value) do val
        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        steering.rotational_component.x_rotational_radius = val
 

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end


function event_slider_param_θz_radius(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_param.sg_param.sliders[2].value) do val
        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        steering.rotational_component.z_rotational_radius = val

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end

function event_slider_param_tierod(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_param.sg_param.sliders[3].value) do val
        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        steering.tie_rod.length = val
        #steering.TrackLever.length = 

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end


function event_slider_param_tracklever(interaction_lyt::InteractionLyt,
                                            θ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info


    on(section_param.sg_param.sliders[4].value) do val
        θx = section_angle.sg_θ.sliders[1].value.val
        θy = section_angle.sg_θ.sliders[2].value.val
        θz = section_angle.sg_θ.sliders[3].value.val


        steering.track_lever.length = val

        # Calculation
        update_geometry!((θx,θy,θz),section_plot,steering, suspension)
        obj = ackermann_deviation((θx,θy,θz),chassis,steering,suspension)
        ratio = ackermannratio((θx,θy,θz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

        if section_plot_settings.menu.selection.val == "geometry"
            section_plot.ax_geom.title = "Steering geometry for (θx, θy, θz) = ($θx,$θy,$θz)"
        end

        if section_plot_settings.menu.selection.val == "radii"
            section_plot.ax_radii.title = "Radii for (θx, θy, θz) = ($θx,$θy,$θz)"
            section_plot.obs_radii_θz[] = steering_radii_θz(θx,θy,θz_max,chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "ackermannratio"
            section_plot.ax_ratio.title = "ackermannratio for (θx, θy, θz) = ($θx,$θy,$θz)"
            ratio_θz = ackermannratio_θz(θx,θy,θz_max,chassis, steering, suspension)
            section_plot.obs_ratio_θz[] = ratio_θz
            section_plot.obs_ratio_min[] = minimum(ratio_θz)
            section_plot.obs_ratio_max[] = maximum(ratio_θz)
        end


        #Updating
        section_info.tb_obj.displayed_string = "objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "ackermannratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "radius: $(round(radius, digits=2))mm"
    end

end



#TODO plot exeposition tracking for saving
function geteyepostion(interaction_lyt::InteractionLyt,
                        θ_max, 
                        chassis::Chassis, 
                        steering::Steering,
                        suspension::Suspension)

    θx_max, θy_max, θz_max = θ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info



    # Interaktive 3D-Kamera aktivieren
    cam = cam3d!(section_plot.ax_geom.scene)

    # Zeige die Kamera-Position
    @info "Position" cam.eyeposition[]
    @info "Look-at target" cam.lookat[]
    @info "Up vector" cam.upvector[]
    
end