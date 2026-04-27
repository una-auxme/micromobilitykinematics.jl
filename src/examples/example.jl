using micromobilitykinematics
using micromobilitykinematics: Chassis, Steering, Suspension, getValue, lounch_gui

max_angle_config = (15.0, 1.0, 35.0)

default_parameters = (
    57.4050864963812,
    100.0000009999905,
    109.196240211308,
    229.7228503290388,
)

if "--optimize" in ARGS
    lower_border = (50.0, 50.0, 70.0, 195.0)
    upper_border = (100.0, 100.0, 200.0, 260.0)

    println("Searching feasible start parameters...")
    start_parameters = random_search(upper_border, lower_border, max_angle_config)
    println("Start parameters: ", start_parameters)

    println("Optimizing steering geometry...")
    opt = optim_over_range(start_parameters..., max_angle_config)

    println("Input parameters: ", opt.input)
    println("Optimized parameters: ", getValue(opt.steering))
    println("Optimized angles: ", opt.θ)
    println("Objective: ", opt.objective)
    println("Status: ", opt.status)

    steering = opt.steering
else
    println("Using known feasible parameters.")
    println("Run with --optimize to perform random search and optimization.")
    steering = Steering(default_parameters...)
end

suspension = Suspension((30.0, 30.0))
chassis = Chassis()

angle_config = (0.0, 1.0, 0.0)
update!(angle_config, steering, suspension)

println("Updated steering state at angle ", angle_config)
println("Inner wheel angle: ", steering.δi)
println("Outer wheel angle: ", steering.δo)

if "--gui" in ARGS
    # Loading GLMakie activates the package extension that implements lounch_gui.
    using GLMakie

    gui_angle_limits = (15.0, 5.0, 35.0)
    lounch_gui(gui_angle_limits, chassis, steering, suspension; path = @__DIR__)
end
