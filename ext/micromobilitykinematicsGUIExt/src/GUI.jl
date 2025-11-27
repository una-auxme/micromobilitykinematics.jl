"""
    GUI_steering(θ_max, 
                  chassis::Chassis, 
                  steering::Steering,
                  suspension::Suspension)

Initializes the interactive steering GUI and registers all associated event handlers.

# Arguments
- `args...`: A variadic argument list passed to all subcomponents, expected to include:
  - `θ_max`: Tuple of maximum angle values `(θx_max, θy_max, θz_max)`
  - `chassis::Chassis`: The chassis system
  - `steering::Steering`: The steering system
  - `suspension::Suspension`: The suspension system

# Description
This function sets up the complete steering user interface by:
- Creating the layout (`interactionlyt`)
- Registering all angle slider callbacks (`event_slider_θ`)
- Setting up the plot settings menu logic (`event_menu_plot_settings`)
- Enabling functionality to save current plots (`event_btn_save`)
- Enabling functionality to save all plots in one click (`event_btn_save_all`)

This acts as the main entry point for initializing the interactive GUI system for steering dynamics and visualization.

# Returns
Nothing. Sets up UI and reactive event systems via side effects.
"""
function GUI_steering(args...; path = @__DIR__)

  args[3].init_steering.θx = args[3].θx
  args[3].init_steering.θy = args[3].θy
  args[3].init_steering.θz = args[3].θz

  interaction_lyt = interactionlyt(args...; path = path)
  event_slider_θ(interaction_lyt, args...)
  event_menu_plot_settings(interaction_lyt, args...)                  
  event_btn_save(interaction_lyt, args...)
  event_btn_save_all(interaction_lyt, args...)
  event_slider_right_compression(interaction_lyt, args...)
  event_slider_left_compression(interaction_lyt, args...)
  event_btn_reset(interaction_lyt, args...)
  event_slider_param_θx_radius(interaction_lyt, args...)
  event_slider_param_θz_radius(interaction_lyt, args...)
  event_slider_param_tierod(interaction_lyt, args...)
  event_slider_param_tracklever(interaction_lyt, args...)
  event_XML_Export(interaction_lyt,args...)
end



