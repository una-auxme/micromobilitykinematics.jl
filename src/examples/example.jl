
lower_bourder = (50.0, 50.0, 70.0, 195.0)  
upper_bourder = (100.0, 100.0, 200.0, 260.0)
max_angleConfig = (15.0, 1.0, 35.0)

x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length = random_search(upper_bourder, lower_bourder, max_angleConfig)

opt = optim_over_range(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length, max_angleConfig)

opt.input
opt.steering
opt.θ
opt.objective
opt.status


x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length = getValue(opt.steering)

st = Steering(57.4050864963812, 100.0000009999905, 109.196240211308, 229.7228503290388)

st = opt.steering

suspension = Suspension((30,30))
# steering setting
angleConfig = (0.0,1.0,0.0)
suspensionkinematics!(suspension)

chassis = Chassis()

update!(angleConfig, st, suspension)


θ_max = (15.0,5.0,35.0)
GUI_steering(θ_max, chassis , st, suspension)



