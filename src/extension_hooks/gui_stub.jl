"""
    launch_gui(ϕ_max, 
                  chassis::Chassis, 
                  steering::Steering,
                  suspension::Suspension)

Initializes the interactive steering GUI and registers all associated event handlers.

# Arguments
- `args...`: A variadic argument list passed to all subcomponents, expected to include:
  - `ϕ_max`: Tuple of maximum angle values `(ϕx_max, ϕy_max, ϕz_max)`
  - `chassis::Chassis`: The chassis system
  - `steering::Steering`: The steering system
  - `suspension::Suspension`: The suspension system

# Keywords
-`path`: String to DIC 

# Description
This function sets up the complete steering user interface by:
- Creating the layout (`interactionlyt`)
- Registering all angle slider callbacks (`event_slider_ϕ`)
- Setting up the plot settings menu logic (`event_menu_plot_settings`)
- Enabling functionality to save current plots (`event_btn_save`)
- Enabling functionality to save all plots in one click (`event_btn_save_all`)

This acts as the main entry point for initializing the interactive GUI system for steering dynamics and visualization.

# Returns
Nothing. Sets up UI and reactive event systems via side effects.
"""
function launch_gui end

const lounch_gui = launch_gui
