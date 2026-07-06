

"""
    event_slider_φx(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    tiggers slider event for φx

Registers and handles the slider event for the φx steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the φx slider (the first slider in the `section_angle.sg_ϕ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(φx, φy, φz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"Geometry"`, `"Radii"`, or `"Ackermann ratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_ϕx(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)
    
    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error



    on(section_angle.sg_ϕ.sliders[1].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check

         @safe_ui steering suspension interaction_lyt begin
            ϕx = val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end

            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end

"""
    event_slider_φy(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers and handles the slider event for the φy steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the φy slider (the second slider in the `section_angle.sg_ϕ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(φx, φy, φz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"Geometry"`, `"Radii"`, or `"Ackermann ratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_ϕy(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_angle.sg_ϕ.sliders[2].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            


            # 

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Steering vs. wheel angles"
                section_plot.ax_ϕ_vs_δ_surface.title = varphi_vs_delta_title(ϕx_max, ϕy, ϕz_max, suspension)
                section_plot.obs_ϕ_vs_δi_surface[] = ax_ϕ_vs_δi(steering, suspension, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ϕ_vs_δo_surface[] = ax_ϕ_vs_δo(steering, suspension, (ϕx_max, ϕy, ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end


            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            # Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end
end


"""
    event_slider_φz(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers and handles the slider event for the φz steering angle in the UI.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing UI components, plots, and interactive elements.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` representing the maximum values for each rotation angle.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system object.
- `suspension::Suspension`: The suspension system object.

# Description
This function sets up an event listener for the φz slider (the third slider in the `section_angle.sg_ϕ` array). When the slider value changes, the system:
- Updates the geometry based on current steering angles `(φx, φy, φz)`.
- Computes key metrics such as steering objective, Ackermann ratio, and turning radius.
- Dynamically updates plot titles and observable values depending on the selected plot mode (`"Geometry"`, `"Radii"`, or `"Ackermann ratio"`).
- Displays updated information about steering angles, objective, and turning radius in the info section.

# Returns
Nothing. The function relies on UI event callbacks to update the system state.
"""
function event_slider_ϕz(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error
    
    on(section_angle.sg_ϕ.sliders[3].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = val
            #

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx, ϕy, ϕz_max, chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Steering vs. wheel angles"
                section_plot.ax_ϕ_vs_δ_surface.title = varphi_vs_delta_title(ϕx_max, ϕy, ϕz_max, suspension)
                section_plot.obs_ϕ_vs_δi_surface[] = ax_ϕ_vs_δi(steering, suspension, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ϕ_vs_δo_surface[] = ax_ϕ_vs_δo(steering, suspension, (ϕx_max, ϕy, ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end



            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            # Updating 
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end

"""
    event_slider_ϕ(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


Initializes all ϕ angle slider event handlers (φx, φy, φz) in the user interface.

# Arguments
- `args...`: Variadic arguments passed to each of the individual event handler functions:
  - `interaction_lyt::InteractionLyt`
  - `ϕ_max`: Tuple of maximum angle values `(φx_max, φy_max, φz_max)`
  - `chassis::Chassis`
  - `steering::Steering`
  - `suspension::Suspension`

# Description
This convenience function initializes the event listeners for all three rotation angle sliders:
- `event_slider_φx` for pitch/roll angle,
- `event_slider_φy` for yaw angle,
- `event_slider_φz` for roll/pitch angle.

Each handler updates the vehicle model, calculations, and corresponding plots when the slider value changes.

# Returns
Nothing. Registers the slider event callbacks via side effects.
"""
function event_slider_ϕ(args...)
    event_slider_ϕx(args...)
    event_slider_ϕy(args...)
    event_slider_ϕz(args...)
end


"""
    event_slider_left_compression(interaction_lyt::InteractionLyt,
                                    ϕ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)

Initializes the event handler for the left damper compression slider in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: Layout container holding all relevant UI components.
- `ϕ_max`: Tuple of maximum angle values `(φx_max, φy_max, φz_max)` used for plotting limits and calculations.
- `chassis::Chassis`: Vehicle chassis representation.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension system model, including damper compression states.

# Description
This function registers an event listener for the left suspension damper compression slider. When the user changes the slider value, the following occurs:

- The new compression value is applied to the suspension model.
- The current orientation angles `(φx, φy, φz)` are read from the interface.
- Vehicle geometry is recalculated based on updated suspension and steering inputs.
- Various plots are updated depending on the selected plot mode:
  - Geometry plots (`"Geometry"`)
  - Turning radius plots (`"Radii"`)
  - Ackermann ratio plots (`"Ackermann ratio"`)
- Key information (objective value, Ackermann ratio, steering angles, turning radius) is displayed and refreshed in the interface.

# Returns
Nothing. Registers the event handler as a side effect.
"""
function event_slider_left_compression(interaction_lyt::InteractionLyt,
                                        ϕ_max, 
                                        chassis::Chassis, 
                                        steering::Steering,
                                        suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error

    on(section_damper.sg_compr.sliders[1].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            left_compr = val
            right_compr = section_damper.sg_compr.sliders[2].value.val

            suspension.damper[1].compression = left_compr
            suspension.damper[2].compression = right_compr


            

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            if section_plot_settings.menu.selection.val == "Steering vs. wheel angles"
                section_plot.ax_ϕ_vs_δ_surface.title = varphi_vs_delta_title(ϕx_max, ϕy, ϕz_max, suspension)
                section_plot.obs_ϕ_vs_δi_surface[] = ax_ϕ_vs_δi(steering, suspension, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ϕ_vs_δo_surface[] = ax_ϕ_vs_δo(steering, suspension, (ϕx_max, ϕy, ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end



            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end
end


"""
    event_slider_right_compression(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

Initializes the event handler for the right damper compression slider in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: Layout container holding all relevant UI components.
- `ϕ_max`: Tuple of maximum angle values `(φx_max, φy_max, φz_max)` used for plotting limits and calculations.
- `chassis::Chassis`: Vehicle chassis representation.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension system model, including damper compression states.

# Description
This function registers an event listener for the right suspension damper compression slider. When the user changes the slider value, the following occurs:

- The new compression value is applied to the suspension model.
- The current orientation angles `(φx, φy, φz)` are read from the interface.
- Vehicle geometry is recalculated based on updated suspension and steering inputs.
- Various plots are updated depending on the selected plot mode:
  - Geometry plots (`"Geometry"`)
  - Turning radius plots (`"Radii"`)
  - Ackermann ratio plots (`"Ackermann ratio"`)
- Key information (objective value, Ackermann ratio, steering angles, turning radius) is displayed and refreshed in the interface.

# Returns
Nothing. Registers the event handler as a side effect.
"""
function event_slider_right_compression(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error

    on(section_damper.sg_compr.sliders[2].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            left_compr = section_damper.sg_compr.sliders[1].value.val
            right_compr = val

            suspension.damper[1].compression = left_compr
            suspension.damper[2].compression = right_compr

            

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            if section_plot_settings.menu.selection.val == "Steering vs. wheel angles"
                section_plot.ax_ϕ_vs_δ_surface.title = varphi_vs_delta_title(ϕx_max, ϕy, ϕz_max, suspension)
                section_plot.obs_ϕ_vs_δi_surface[] = ax_ϕ_vs_δi(steering, suspension, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ϕ_vs_δo_surface[] = ax_ϕ_vs_δo(steering, suspension, (ϕx_max, ϕy, ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end


            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)
            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end
            

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end
end


"""
    event_slider_compression(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)

Initializes both left and right damper compression slider event handlers in the user interface.

# Arguments
- `args...`: Variadic arguments passed to the respective handler functions:
  - `interaction_lyt::InteractionLyt`: UI layout container.
  - `ϕ_max`: Tuple of maximum rotation angles `(φx_max, φy_max, φz_max)`.
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
                                    ϕ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)

Registers an event listener for the plot settings menu and updates plot visibility,
sliders, and computed values accordingly.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object containing UI elements such as plots, sliders, and menus.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` specifying the maximum values for the steering angles.
- `chassis::Chassis`: The chassis model of the vehicle.
- `steering::Steering`: The steering system model.
- `suspension::Suspension`: The suspension system model.

# Description
This function handles changes to the selection in the plot settings menu. Depending on the selected mode (`"Geometry"`, `"Radii"`, `"Ackermann ratio"`, or `"Ackermann ratio surface plot"`), it:
- Shows or hides the appropriate plot axes.
- Toggles the visibility of angle sliders (φx, φy, φz) accordingly.
- Computes and updates the relevant observables (`obs_radii_φz`, `obs_ratio_φz`, `obs_ratio_surface`) used for plotting.
- Adjusts the display based on current slider values and system state.

# Returns
Nothing. Updates the UI and observable values as a side effect of menu interactions.
"""
function event_menu_plot_settings(interaction_lyt::InteractionLyt,
                                    ϕ_max, 
                                    chassis::Chassis, 
                                    steering::Steering,
                                    suspension::Suspension)



    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error

    on(section_plot_settings.menu.selection) do sel

        if sel == "Geometry"

            update_layout_visibility!(interaction_lyt; 
                                        geom = true, 
                                        sg_ϕx = true, 
                                        sg_ϕy = true, 
                                        sg_ϕz = true, 
                                        comprL = true, 
                                        comprR = true)
        end


        if sel == "Radii"

            update_layout_visibility!(interaction_lyt; 
                                        radii = true,
                                        sg_ϕx = true, 
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
        end


        if sel == "Ackermann ratio"

            update_layout_visibility!(interaction_lyt; 
                                        ratio = true, 
                                        sg_ϕx = true, 
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            update_ratio_ϕz_plot!(section_plot, ϕx, ϕy, ϕz, ϕz_max, chassis, steering, suspension)
        end

        if sel == "Ackermann ratio φx sweep"

            update_layout_visibility!(interaction_lyt; 
                                        ratio_ϕx = true, 
                                        sg_ϕy = true,
                                        sg_ϕz = true,
                                        comprL = true, 
                                        comprR = true)

            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
        end

        if sel == "Ackermann ratio surface plot"

            update_layout_visibility!(interaction_lyt; 
                                        ratio_surf = true,
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            update_ratio_surface_plot!(section_plot, ϕy, ϕ_max, chassis, steering, suspension)
        end

        if sel == "Steering vs. wheel angles"

            update_layout_visibility!(interaction_lyt; 
                                        ϕ_vs_δ = true,
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            section_plot.ax_ϕ_vs_δ_surface.title = varphi_vs_delta_title(ϕx_max, ϕy, ϕz_max, suspension)
            section_plot.obs_ϕ_vs_δi_surface[] = ax_ϕ_vs_δi(steering, suspension, (ϕx_max, ϕy, ϕz_max))
            section_plot.obs_ϕ_vs_δo_surface[] = ax_ϕ_vs_δo(steering, suspension, (ϕx_max, ϕy, ϕz_max))
        end

        if sel == "Ackermann deviation"

            update_layout_visibility!(interaction_lyt; 
                                        deviation = true,
                                        sg_ϕx = true, 
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            section_plot.ax_deviation.title = ackermann_deviation_title(ϕx_max, ϕy, ϕz_max, suspension)
            section_plot.obs_deviation_ϕz[] = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
        end

         if sel == "Ackermann deviation surface"

            update_layout_visibility!(interaction_lyt; 
                                        deviation_surf = true,
                                        sg_ϕy = true,
                                        comprL = true, 
                                        comprR = true)

            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            chassis_copy = deepcopy(chassis)
            steering_copy = deepcopy(steering)
            suspension_copy = deepcopy(suspension)

            section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
        end

        if sel == "Compression vs. wheel angles"

            update_layout_visibility!(interaction_lyt; 
                                        compr_vs_δ = true, 
                                        sg_ϕx = true, 
                                        sg_ϕy = true, 
                                        sg_ϕz = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            update_compr_vs_delta_surface!(section_plot, (ϕx, ϕy, ϕz), steering, suspension)
        end

        if sel == "Left wheel Δδ vs. compression"

            update_layout_visibility!(interaction_lyt;
                                        left_wheel_delta = true,
                                        sg_ϕx = true,
                                        sg_ϕy = true,
                                        comprR = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
        end

        if sel == "Wheel center path"

            update_layout_visibility!(interaction_lyt;
                                        wheel_center_path = true)

            update_wheel_center_path_plot!(section_plot, steering, suspension)
        end

        if sel == "Wheel center surface"

            update_layout_visibility!(interaction_lyt;
                                        wheel_center_surface = true,
                                        sg_ϕx = true,
                                        sg_ϕy = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val

            update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
        end

        if sel == "Track width"

            update_layout_visibility!(interaction_lyt;
                                        track_width = true)

            update_track_width_plot!(section_plot, steering, suspension)
        end

        if sel == "Damper motion ratio"

            update_layout_visibility!(interaction_lyt;
                                        motion_ratio = true)

            update_motion_ratio_plot!(section_plot, steering, suspension)
        end

        if sel == "Roll kinematics"

            update_layout_visibility!(interaction_lyt;
                                        roll_kinematics = true,
                                        sg_ϕx = true,
                                        sg_ϕy = true,
                                        sg_ϕz = true)

            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val

            update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
        end

    end
end


function event_ackermann_ratio_signed(interaction_lyt::InteractionLyt,
                                        ϕ_max, 
                                        chassis::Chassis, 
                                        steering::Steering,
                                        suspension::Suspension)

    section_plot_settings = interaction_lyt.section_plot_settings

    on(section_plot_settings.cb_signed_ratio.checked) do signed
        set_ackermann_ratio_signed!(signed)

        @safe_ui steering suspension interaction_lyt begin
            update_current_ackermann_ratio_views!(interaction_lyt, ϕ_max, chassis, steering, suspension)
        end
    end

    nothing
end



"""
    event_btn_save(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers a callback for the 'Save' button to export the currently selected plot view
based on the active steering angles.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout object managing the user interface, including plots, controls, and buttons.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` representing the maximum allowable values for the steering angles.
- `chassis::Chassis`: The vehicle chassis model.
- `steering::Steering`: The vehicle's steering system.
- `suspension::Suspension`: The vehicle's suspension system.

# Description
When the save button is clicked, this function:
- Reads the current values of the steering angles `(φx, φy, φz)` from the sliders.
- Determines which plot type is currently selected via the menu.
- Generates the appropriate plot (geometry, Ackermann ratio over φz, or surface plot).
- Saves the figure as a PNG file using a filename that includes the angle values (where applicable).
- Displays the original figure again after saving.

Currently, no file is saved if the `"Radii"` plot type is selected.

# Returns
Nothing. The function performs file-saving and UI updates as side effects of the button click event.
"""
function event_btn_save(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)


    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    base_path = interaction_lyt.path

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error

    on(section_plot_settings.btn_save.clicks) do n
        mkpath(base_path)

        ϕx = section_angle.sg_ϕ.sliders[1].value.val
        ϕy = section_angle.sg_ϕ.sliders[2].value.val
        ϕz = section_angle.sg_ϕ.sliders[3].value.val

        ϕ = (ϕx, ϕy, ϕz)

        if section_plot_settings.menu.selection.val == "Geometry"
            # Deine Plot-Funktion: sollte z. B. (fig=..., geo_ax=Axis3(...)) liefern
            fig_geo = geometry_plot(ϕ, steering, suspension)

            ax_geo = first(values(fig_geo.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            ax_geo.azimuth[]   = section_plot.ax_geom.azimuth[]
            ax_geo.elevation[] = section_plot.ax_geom.elevation[]
            #fig_geo.geo_ax.zoom[]      = section_plot.ax_geom.zoom[]  # optional

            file_path = joinpath(base_path, "geometry(φx,φy,φz)=($ϕx,$ϕy,$ϕz).png")
            GLMakie.save(file_path,fig_geo; px_per_unit = 20)
            GLMakie.display(fig)

        end

        if section_plot_settings.menu.selection.val == "Radii"
            # Deine Plot-Funktion: sollte z. B. (fig=..., geo_ax=Axis3(...)) liefern
            fig_radii = radii_plot(ϕx,ϕy,ϕz_max,chassis, steering, suspension)

            ax_geo = first(values(fig_radii.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            #ax_geo.azimuth[]   = section_plot.ax_geom.azimuth[]
            #ax_geo.elevation[] = section_plot.ax_geom.elevation[]
            #fig_geo.geo_ax.zoom[]      = section_plot.ax_geom.zoom[]  # optional

            file_path = joinpath(base_path, "radii(φx,φy,φz)=($ϕx,$ϕy,$ϕz).png")
            GLMakie.save(file_path,fig_radii; px_per_unit = 20)
            GLMakie.display(fig)

        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio"
            fig_ratio = ackermannratio_ϕz_plot(ϕx,ϕy,ϕz_max, chassis, steering, suspension)

            ax_ratio = first(values(fig_ratio.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            #ax_ratio.azimuth[]   = section_plot.ax_ratio.azimuth[]
            #ax_ratio.elevation[] = section_plot.ax_ratio.elevation[]

            file_path = joinpath(base_path, "ackermannratio(φx,φy,φz)=($ϕx,$ϕy,φz).png")
            GLMakie.save(file_path,fig_ratio)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
            fig_ratio_ϕx = ackermannratio_ϕx_plot(ϕx_max, ϕy, ϕz, chassis, steering, suspension)

            file_path = joinpath(base_path, "ackermannratio_φx_sweep.png")
            GLMakie.save(file_path, fig_ratio_ϕx)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
            fig_ratio_surface = ratio_surface_plot(ϕy, ϕ_max, chassis, steering, suspension)

            ax_ratio_surface = first(values(fig_ratio_surface.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            ax_ratio_surface.azimuth[]   = section_plot.ax_ratio_surface.azimuth[]
            ax_ratio_surface.elevation[] = section_plot.ax_ratio_surface.elevation[]

            file_path = joinpath(base_path, "ackermannratio_surface_plot.png")
            GLMakie.save(file_path,fig_ratio_surface)
            GLMakie.display(fig)
        end


        if section_plot_settings.menu.selection.val == "Steering vs. wheel angles"
            fig_ϕ_vs_δ_surface = ϕ_vs_δ_plot(ϕy, ϕ_max, steering, suspension)

            ax_ϕ_vs_δ_surface = first(values(fig_ϕ_vs_δ_surface.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            ax_ϕ_vs_δ_surface.azimuth[]   = section_plot.ax_ϕ_vs_δ_surface.azimuth[]
            ax_ϕ_vs_δ_surface.elevation[] = section_plot.ax_ϕ_vs_δ_surface.elevation[]

            file_path = joinpath(base_path, "steering_vs_wheel_angles.png")
            GLMakie.save(file_path,fig_ϕ_vs_δ_surface)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Ackermann deviation"
            fig_deviation = deviation_plot(ϕx, ϕy, ϕz_max, chassis, steering, suspension)

            ax_deviation = first(values(fig_deviation.content))

            # Variante A: nur Az/El (einfach & ausreichend in vielen Fällen)
            #ax_deviation.azimuth[]   = section_plot.ax_deviation.azimuth[]
            #ax_deviation.elevation[] = section_plot.ax_deviation.elevation[]

            file_path = joinpath(base_path, "ackermann_deviation.png")
            GLMakie.save(file_path, fig_deviation)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
            fig_deviation_surface = deviation_surface_plot(ϕy, ϕ_max, chassis, steering, suspension)

            for content in values(fig_deviation_surface.content)
                if content isa Axis3
                    content.azimuth[] = section_plot.ax_deviation_surface.azimuth[]
                    content.elevation[] = section_plot.ax_deviation_surface.elevation[]
                    break
                end
            end

            file_path = joinpath(base_path, "ackermann_deviation_surface.png")
            GLMakie.save(file_path,fig_deviation_surface)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
            fig_compr_vs_δ = compr_vs_δ_plot(ϕx, ϕy, ϕz, steering, suspension)

            for content in values(fig_compr_vs_δ.content)
                if content isa Axis3
                    content.azimuth[] = section_plot.ax_compr_vs_δ.azimuth[]
                    content.elevation[] = section_plot.ax_compr_vs_δ.elevation[]
                    break
                end
            end

            file_path = joinpath(base_path, "compression_vs_wheel_angles.png")
            GLMakie.save(file_path,fig_compr_vs_δ)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
            fig_left_wheel_delta = left_wheel_delta_plot(ϕx, ϕy, ϕz_max, steering, suspension)

            for content in values(fig_left_wheel_delta.content)
                if content isa Axis3
                    content.azimuth[] = section_plot.ax_left_wheel_delta.azimuth[]
                    content.elevation[] = section_plot.ax_left_wheel_delta.elevation[]
                    break
                end
            end

            file_path = joinpath(base_path, "left_wheel_delta_vs_compression.png")
            GLMakie.save(file_path,fig_left_wheel_delta)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Wheel center path"
            fig_wheel_center_path = wheel_center_path_plot(steering, suspension)

            for content in values(fig_wheel_center_path.content)
                if content isa Axis3
                    content.azimuth[] = section_plot.ax_wheel_center_path.azimuth[]
                    content.elevation[] = section_plot.ax_wheel_center_path.elevation[]
                    break
                end
            end

            file_path = joinpath(base_path, "wheel_center_path.png")
            GLMakie.save(file_path, fig_wheel_center_path)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Wheel center surface"
            fig_wheel_center_surface = wheel_center_surface_plot(ϕx, ϕy, ϕz_max, steering, suspension)

            for content in values(fig_wheel_center_surface.content)
                if content isa Axis3
                    content.azimuth[] = section_plot.ax_wheel_center_surface.azimuth[]
                    content.elevation[] = section_plot.ax_wheel_center_surface.elevation[]
                    break
                end
            end

            file_path = joinpath(base_path, "wheel_center_surface.png")
            GLMakie.save(file_path, fig_wheel_center_surface)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Track width"
            fig_track_width = track_width_plot(steering, suspension)

            file_path = joinpath(base_path, "track_width.png")
            GLMakie.save(file_path, fig_track_width)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Damper motion ratio"
            fig_motion_ratio = motion_ratio_plot(steering, suspension)

            file_path = joinpath(base_path, "damper_motion_ratio.png")
            GLMakie.save(file_path, fig_motion_ratio)
            GLMakie.display(fig)
        end

        if section_plot_settings.menu.selection.val == "Roll kinematics"
            fig_roll_kinematics = roll_kinematics_plot((ϕx, ϕy, ϕz), chassis, steering, suspension)

            file_path = joinpath(base_path, "roll_kinematics.png")
            GLMakie.save(file_path, fig_roll_kinematics)
            GLMakie.display(fig)
        end
    end
end


"""
    event_btn_save_all(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Registers a callback for the 'Save All' button to export all major plot views
(geometry, Ackermann ratio over φz, and Ackermann ratio surface plot)
based on the current steering angle configuration.

# Arguments
- `interaction_lyt::InteractionLyt`: The layout structure containing the full UI with figures, sliders, and controls.
- `ϕ_max`: A tuple `(φx_max, φy_max, φz_max)` defining the upper bounds for steering angles.
- `chassis::Chassis`: The vehicle's chassis system.
- `steering::Steering`: The steering system configuration.
- `suspension::Suspension`: The suspension model of the vehicle.

# Description
On button click, this function:
- Reads the current values of the steering angles `(φx, φy, φz)` from the UI sliders.
- Generates the following plots:
  - Steering geometry plot for `(φx, φy, φz)`
  - Ackermann ratio plot over varying `φz`
  - Ackermann ratio surface plot over the full angle space
- Saves each figure to disk with an appropriate filename.
- Ensures the original figure remains displayed after saving.

This is a convenience function to quickly export all relevant analysis visualizations in a single action.

# Returns
Nothing. Executes saving and visualization as side effects triggered by the button click event.
"""
function event_btn_save_all(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    base_path = interaction_lyt.path

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_plot_settings.btn_save_all.clicks) do n
        mkpath(base_path)

        ϕx = section_angle.sg_ϕ.sliders[1].value.val
        ϕy = section_angle.sg_ϕ.sliders[2].value.val
        ϕz = section_angle.sg_ϕ.sliders[3].value.val

        ϕ = (ϕx, ϕy, ϕz)

        ####
        fig_geo = geometry_plot(ϕ, steering, suspension)

        ax_geo = first(values(fig_geo.content))

        ax_geo.azimuth[]   = section_plot.ax_geom.azimuth[]
        ax_geo.elevation[] = section_plot.ax_geom.elevation[]

        file_path = joinpath(base_path, "geometry(φx,φy,φz)=($ϕx,$ϕy,$ϕz).png")
        GLMakie.save(file_path,fig_geo; px_per_unit = 20)




        ###
        fig_radii = radii_plot(ϕx,ϕy,ϕz_max,chassis, steering, suspension)

        ax_geo = first(values(fig_radii.content))

        file_path = joinpath(base_path, "radii(φx,φy,φz)=($ϕx,$ϕy,$ϕz).png")
        GLMakie.save(file_path,fig_radii; px_per_unit = 20)


        ###
        fig_ratio = ackermannratio_ϕz_plot(ϕx,ϕy,ϕz_max, chassis, steering, suspension)

        ax_ratio = first(values(fig_ratio.content))

        file_path = joinpath(base_path, "ackermannratio(φx,φy,φz)=($ϕx,$ϕy,φz).png")
        GLMakie.save(file_path,fig_ratio)


        ###
        fig_ratio_ϕx = ackermannratio_ϕx_plot(ϕx_max, ϕy, ϕz, chassis, steering, suspension)

        file_path = joinpath(base_path, "ackermannratio_φx_sweep.png")
        GLMakie.save(file_path, fig_ratio_ϕx)


        ###
        fig_ratio_surface = ratio_surface_plot(ϕy, ϕ_max, chassis, steering, suspension)

        ax_ratio_surface = first(values(fig_ratio_surface.content))


        ax_ratio_surface.azimuth[]   = section_plot.ax_ratio_surface.azimuth[]
        ax_ratio_surface.elevation[] = section_plot.ax_ratio_surface.elevation[]

        file_path = joinpath(base_path, "ackermannratio_surface_plot.png")
        GLMakie.save(file_path,fig_ratio_surface)


        ###
        fig_ϕ_vs_δ_surface = ϕ_vs_δ_plot(ϕy, ϕ_max, steering, suspension)

        ax_ϕ_vs_δ_surface = first(values(fig_ϕ_vs_δ_surface.content))

        ax_ϕ_vs_δ_surface.azimuth[]   = section_plot.ax_ϕ_vs_δ_surface.azimuth[]
        ax_ϕ_vs_δ_surface.elevation[] = section_plot.ax_ϕ_vs_δ_surface.elevation[]

        file_path = joinpath(base_path, "steering_vs_wheel_angles.png")
        GLMakie.save(file_path,fig_ϕ_vs_δ_surface)


        ###
        fig_deviation = deviation_plot(ϕx, ϕy, ϕz_max, chassis, steering, suspension)

        ax_deviation = first(values(fig_deviation.content))

        file_path = joinpath(base_path, "ackermann_deviation.png")
        GLMakie.save(file_path, fig_deviation)


        ###
        fig_deviation_surface = deviation_surface_plot(ϕy, ϕ_max, chassis, steering, suspension)

        for content in values(fig_deviation_surface.content)
            if content isa Axis3
                content.azimuth[] = section_plot.ax_deviation_surface.azimuth[]
                content.elevation[] = section_plot.ax_deviation_surface.elevation[]
                break
            end
        end

        file_path = joinpath(base_path, "ackermann_deviation_surface.png")
        GLMakie.save(file_path,fig_deviation_surface)


        ###
        fig_compr_vs_δ = compr_vs_δ_plot(ϕx, ϕy, ϕz, steering, suspension)

        for content in values(fig_compr_vs_δ.content)
            if content isa Axis3
                content.azimuth[] = section_plot.ax_compr_vs_δ.azimuth[]
                content.elevation[] = section_plot.ax_compr_vs_δ.elevation[]
                break
            end
        end

        file_path = joinpath(base_path, "compression_vs_wheel_angles.png")
        GLMakie.save(file_path,fig_compr_vs_δ)

        ###
        fig_left_wheel_delta = left_wheel_delta_plot(ϕx, ϕy, ϕz_max, steering, suspension)

        for content in values(fig_left_wheel_delta.content)
            if content isa Axis3
                content.azimuth[] = section_plot.ax_left_wheel_delta.azimuth[]
                content.elevation[] = section_plot.ax_left_wheel_delta.elevation[]
                break
            end
        end

        file_path = joinpath(base_path, "left_wheel_delta_vs_compression.png")
        GLMakie.save(file_path,fig_left_wheel_delta)

        ###
        fig_wheel_center_path = wheel_center_path_plot(steering, suspension)

        for content in values(fig_wheel_center_path.content)
            if content isa Axis3
                content.azimuth[] = section_plot.ax_wheel_center_path.azimuth[]
                content.elevation[] = section_plot.ax_wheel_center_path.elevation[]
                break
            end
        end

        file_path = joinpath(base_path, "wheel_center_path.png")
        GLMakie.save(file_path, fig_wheel_center_path)


        ###
        fig_wheel_center_surface = wheel_center_surface_plot(ϕx, ϕy, ϕz_max, steering, suspension)

        for content in values(fig_wheel_center_surface.content)
            if content isa Axis3
                content.azimuth[] = section_plot.ax_wheel_center_surface.azimuth[]
                content.elevation[] = section_plot.ax_wheel_center_surface.elevation[]
                break
            end
        end

        file_path = joinpath(base_path, "wheel_center_surface.png")
        GLMakie.save(file_path, fig_wheel_center_surface)


        ###
        fig_track_width = track_width_plot(steering, suspension)

        file_path = joinpath(base_path, "track_width.png")
        GLMakie.save(file_path, fig_track_width)


        ###
        fig_motion_ratio = motion_ratio_plot(steering, suspension)

        file_path = joinpath(base_path, "damper_motion_ratio.png")
        GLMakie.save(file_path, fig_motion_ratio)


        ###
        fig_roll_kinematics = roll_kinematics_plot(ϕ, chassis, steering, suspension)

        file_path = joinpath(base_path, "roll_kinematics.png")
        GLMakie.save(file_path, fig_roll_kinematics)
    end

    GLMakie.display(fig)
end



"""
    event_btn_reset(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

Initializes the reset button event handler in the user interface.

# Arguments
- `interaction_lyt::InteractionLyt`: UI layout container with all relevant interface elements.
- `ϕ_max`: Tuple of maximum rotation angles `(φx_max, φy_max, φz_max)` used in geometry/radius/ratio calculations.
- `chassis::Chassis`: Representation of the vehicle chassis.
- `steering::Steering`: Steering system model.
- `suspension::Suspension`: Suspension model, including damper states.

# Description
This function registers an event listener for the reset button in the UI. When triggered, the handler:

- Resets all rotation angles `(φx, φy, φz)` to zero.
- Resets both left and right damper compression values to a default (30.0 mm).
- Updates the geometry, objective value, Ackermann ratio, and turning radius.
- Refreshes the UI plots and updates all related information displays.
- Ensures that the visual output corresponds to the neutral vehicle state.

# Returns
Nothing. Registers the reset button callback and triggers updates via side effects.
"""
function event_btn_reset(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis, 
                            steering::Steering,
                            suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_plot_settings.btn_reset.clicks) do n

        interaction_lyt.reset_flag = true

        ϕx = steering.init_steering.ϕx
        ϕy = steering.init_steering.ϕy
        ϕz = steering.init_steering.ϕz


        suspension.damper[1].compression = 30.0
        suspension.damper[2].compression = 30.0


        set_close_to!(section_angle.sg_ϕ.sliders[1], ϕx)
        set_close_to!(section_angle.sg_ϕ.sliders[2], ϕy)
        set_close_to!(section_angle.sg_ϕ.sliders[3], ϕz)

        set_close_to!(section_damper.sg_compr.sliders[1], 30.0)
        set_close_to!(section_damper.sg_compr.sliders[2], 30.0)

        set_close_to!(section_param.sg_param.sliders[1], steering.init_steering.ϕx_radius)
        set_close_to!(section_param.sg_param.sliders[2], steering.init_steering.ϕz_radius)
        set_close_to!(section_param.sg_param.sliders[3], steering.init_steering.track_lever_length)
        interaction_lyt.reset_flag = false
        set_close_to!(section_param.sg_param.sliders[4], steering.init_steering.tie_rod_length)




        steering.rotational_component.x_rotational_radius = steering.init_steering.ϕx_radius
        steering.rotational_component.z_rotational_radius = steering.init_steering.ϕz_radius
        steering.track_lever.length = steering.init_steering.track_lever_length
        steering.tie_rod.length = steering.init_steering.tie_rod_length



        if section_plot_settings.menu.selection.val == "Geometry"
            section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
        end

        if section_plot_settings.menu.selection.val == "Radii"
            section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
            section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio"
            section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
            ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
            section_plot.obs_ratio_ϕz[] = ratio_ϕz
            section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
            section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
            update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
            section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
            chassis_copy = deepcopy(chassis)
            steering_copy = deepcopy(steering)
            suspension_copy = deepcopy(suspension)
            ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
            section_plot.obs_ratio_surface[] = ratio_surface
        end

        if section_plot_settings.menu.selection.val == "Ackermann deviation"
            section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
            deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
            section_plot.obs_deviation_ϕz[] = deviation_ϕz
            section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
            section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
        end

        if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
            section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
            chassis_copy = deepcopy(chassis)
            steering_copy = deepcopy(steering)
            suspension_copy = deepcopy(suspension)
            section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
        end

        if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
            section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
            update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
            update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "Wheel center surface"
            update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
        end

        if section_plot_settings.menu.selection.val == "Roll kinematics"
            update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
        end

        # Calculation
        update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
        obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
        ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
        radius = turning_radius(chassis,steering)

         min_radius = nothing 
        try
            min_rad_steeerig = deepcopy(steering)
            min_rad_suspension = deepcopy(suspension)
            MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
            min_radius = turning_radius(chassis, min_rad_steeerig)
        catch err
            #error("Calculation of the minimal radius failed -> $(err.msg)")
            steering.err_info.id = :min_raidus_raield 
            bt = catch_backtrace()
            capture_error!(steering, err, bt)
            steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
            #@error "Min radius failed" exception=(err, catch_backtrace())
            rethrow()
        end


        #Updating
        section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
        section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
        section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
        section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
        section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
        section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
    end

    GLMakie.display(fig)
end


function event_XML_Export(interaction_lyt::InteractionLyt,
                            ϕ_max, 
                            chassis::Chassis,
                            steering::Steering,
                            suspension::Suspension)

    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    path = interaction_lyt.path
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_plot_settings.btn_export.clicks) do n

        exportXML(steering, path = path)
        exportXML(suspension, path = path)

    end
end






function event_slider_param_ϕx_radius(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_param.sg_param.sliders[1].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            steering.rotational_component.x_rotational_radius = val

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end


            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end


function event_slider_param_ϕz_radius(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_param.sg_param.sliders[2].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            steering.rotational_component.z_rotational_radius = val

            

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end


            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end

function event_slider_param_tierod(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_param.sg_param.sliders[4].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
         @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            steering.tie_rod.length = val
            #steering.TrackLever.length = 

           

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end


             # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_raield 
                bt = catch_backtrace()
                capture_error!(steering, err, bt)
                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."
                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end


function event_slider_param_tracklever(interaction_lyt::InteractionLyt,
                                            ϕ_max, 
                                            chassis::Chassis, 
                                            steering::Steering,
                                            suspension::Suspension)
    
    ϕx_max, ϕy_max, ϕz_max = ϕ_max          

    fig = interaction_lyt.fig
    section_plot =  interaction_lyt.section_plot
    section_angle =  interaction_lyt.section_angle
    section_param =  interaction_lyt.section_param
    section_damper =  interaction_lyt.section_damper
    section_plot_settings = interaction_lyt.section_plot_settings
    section_info =  interaction_lyt.section_info
    section_error =  interaction_lyt.section_error


    on(section_param.sg_param.sliders[3].value) do val
        # ---  Temporarily suppress events  ---
        interaction_lyt.reset_flag && return  # <-- Flag-Check
        @safe_ui steering suspension interaction_lyt begin
            ϕx = section_angle.sg_ϕ.sliders[1].value.val
            ϕy = section_angle.sg_ϕ.sliders[2].value.val
            ϕz = section_angle.sg_ϕ.sliders[3].value.val


            steering.track_lever.length = val

            

            if section_plot_settings.menu.selection.val == "Geometry"
                section_plot.ax_geom.title = geometry_title(ϕx, ϕy, ϕz, suspension)
            end

            if section_plot_settings.menu.selection.val == "Radii"
                section_plot.ax_radii.title = radii_title(ϕx, ϕy, ϕz_max, suspension)
                section_plot.obs_radii_ϕz[] = steering_radii_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension) ./ 1000.0
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio"
                section_plot.ax_ratio.title = ackermann_ratio_title(ϕx, ϕy, ϕz, suspension)
                ratio_ϕz = ackermannratio_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_ratio_ϕz[] = ratio_ϕz
                section_plot.obs_ratio_min[] = minimum(ratio_ϕz)
                section_plot.obs_ratio_max[] = maximum(ratio_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio φx sweep"
                update_ratio_ϕx_plot!(section_plot, ϕx_max, ϕy, ϕz, chassis, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Ackermann ratio surface plot"
                section_plot.ax_ratio_surface.title = ackermann_ratio_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                ratio_surface = ackermannratio_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max, ϕy, ϕz_max))
                section_plot.obs_ratio_surface[] = ratio_surface
            end

            
            if section_plot_settings.menu.selection.val == "Ackermann deviation"
                section_plot.ax_deviation.title = ackermann_deviation_title(ϕx, ϕy, ϕz, suspension)
                deviation_ϕz = ackermann_deviation_ϕz(ϕx,ϕy,ϕz_max,chassis, steering, suspension)
                section_plot.obs_deviation_ϕz[] = deviation_ϕz
                section_plot.obs_deviation_min[] = minimum(deviation_ϕz)
                section_plot.obs_deviation_max[] = maximum(deviation_ϕz)
            end

            if section_plot_settings.menu.selection.val == "Ackermann deviation surface"
                section_plot.ax_deviation.title = ackermann_deviation_surface_title(suspension)
                chassis_copy = deepcopy(chassis)
                steering_copy = deepcopy(steering)
                suspension_copy = deepcopy(suspension)
                section_plot.obs_ratio_surface[] = ackermann_deviation_surface(chassis_copy, steering_copy, suspension_copy, (ϕx_max,ϕy,ϕz_max))
            end

            if section_plot_settings.menu.selection.val == "Compression vs. wheel angles"
                section_plot.ax_compr_vs_δ.title = compr_vs_delta_title(ϕx, ϕy, ϕz)
                update_compr_vs_delta_surface!(section_plot, (ϕx,ϕy,ϕz), steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Left wheel Δδ vs. compression"
                update_left_wheel_delta_surface!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Wheel center surface"
                update_wheel_center_surface_plot!(section_plot, ϕx, ϕy, ϕz_max, steering, suspension)
            end

            if section_plot_settings.menu.selection.val == "Roll kinematics"
                update_roll_kinematics_plot!(section_plot, (ϕx, ϕy, ϕz), chassis, steering, suspension)
            end



            # Calculation
            update_geometry!((ϕx,ϕy,ϕz),section_plot,steering, suspension)
            obj = abs(ackermann_deviation((ϕx,ϕy,ϕz),chassis,steering,suspension))
            ratio = ackermannratio((ϕx,ϕy,ϕz),chassis,steering,suspension)
            radius = turning_radius(chassis,steering)

            min_radius = nothing 
            try
                min_rad_steeerig = deepcopy(steering)
                min_rad_suspension = deepcopy(suspension)
                MMK.update!((ϕx_max, ϕy , ϕz_max), min_rad_steeerig, min_rad_suspension)
                min_radius = turning_radius(chassis, min_rad_steeerig)
            catch err
                #error("Calculation of the minimal radius failed -> $(err.msg)")
                steering.err_info.id = :min_raidus_failed
                bt = catch_backtrace()
                capture_error!(steering, err, bt)

                steering.err_info.description = "Failed to compute the minimal turning radius due to an invalid kinematic configuration."

                #@error "Min radius failed" exception=(err, catch_backtrace())
                rethrow()
            end

            #Updating
            section_info.tb_obj.displayed_string = "Objective: $(round(obj, digits=4))mm"
            section_info.tb_ratio.displayed_string = "Ackermann ratio: $(round(ratio, digits=2))%"
            section_info.tb_δi.displayed_string = "δi: $(round(steering.δi, digits=2))°"
            section_info.tb_δo.displayed_string = "δo: $(round(steering.δo, digits=2))°"
            section_info.tb_rad.displayed_string = "Radius: $(round(radius, digits=2))mm"
            section_info.tb_min_rad.displayed_string = "Min. radius: $(round(min_radius, digits=2))mm"
        end
    end

end
