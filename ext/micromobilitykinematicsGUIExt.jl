module micromobilitykinematicsGUIExt

using GLMakie
using micromobilitykinematics
using micromobilitykinematics: Steering, RotationalComponent, TieRod, TrackLever
using micromobilitykinematics: Suspension, Damper, UpperWishbone, WheelMount, Suspension
using micromobilitykinematics: Chassis, Measurements

import micromobilitykinematics: lounch_gui

const MMK = micromobilitykinematics  # optional Alias 


include("micromobilitykinematicsGUIExt/src/GUIcore.jl")
include("micromobilitykinematicsGUIExt/src/calculation.jl")
include("micromobilitykinematicsGUIExt/src/safety.jl")
include("micromobilitykinematicsGUIExt/src/helper.jl")
include("micromobilitykinematicsGUIExt/src/layout.jl")
include("micromobilitykinematicsGUIExt/src/events.jl")
include("micromobilitykinematicsGUIExt/src/plots.jl")
include("micromobilitykinematicsGUIExt/src/io.jl")
include("micromobilitykinematicsGUIExt/src/GUI.jl")

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
function lounch_gui(args...)
    GUI_steering(args...)
end
end 