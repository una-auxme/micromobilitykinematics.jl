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
function GUI_steering(args...)
  interaction_lyt = interactionlyt(args...)
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
end



#steering = Steering(61.0, 74.0, 95.0, 228.0)
steering = Steering(63.77, 100.00, 110.76, 227.22)

# initialisation of the susfpension
suspension = Suspension((30,30))


# steering setting
angleConfig = (0.0,0.0,0.0)

suspensionkinematics!(suspension)


chassis = Chassis()

steeringkinematics!(angleConfig, steering, suspension)

θ_max = (10.0,0.0,35.0)
GUI_steering(θ_max, chassis , steering, suspension)










turning_radius(chassis, steering, suspension, (15,5,35))

radii_matrix = steering_radii(chassis, steering, suspension, (15,15,35))

radii_matrix[1,:]

xs = LinRange(0, 10, 100)
ys = LinRange(0, 15, 100)
zs = [cos(x) * sin(y) for x in xs, y in ys]

surface(xs, ys, zs, axis=(type=Axis3,))



A = [1,2,3]

A[end]

fig = Figure()

ax = Axis(fig[1,1])

txt = text!(ax, "aspfadf")

typeof(txt)

a = Text()

typeof()