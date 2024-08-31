θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (79.0, 112.0, 138.0, 216.0)
θx_,θz_ = (1,20)
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e3,  "max_iter" => 600))

set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 100.0)
@variable(model, 50.0 <= z_rotational_radius <= 100.0)
@variable(model, 70.0 <= track_lever_length <= 100.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

#@variable(model, θx)
#@variable(model, θz)


register(model, :objective, 6, objective, autodiff=true)



################## objective function ##################
@objective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))


################## constraints        ##################
@constarint(model, C, checkConstraints°(θx_max, θz_max) >= 1 )


set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)