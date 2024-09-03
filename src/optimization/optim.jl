θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (71.0, 105.0, 131.0, 218.0)
θx_,θz_ = (1,20)
#model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e-12,  "max_iter" => 600))
model = Model(NLopt.Optimizer)
#verwendung von :LD_MMA funktionier besser
# lokales Minimum bereschnen da wir mit vor random suche bereits ein guten Start wert ermitteln und dannach diesen minimieren.
# jedes werte paar hat vermutlich seinen eigenen besten wert.

set_optimizer_attribute(model, "algorithm", :LD_MMA)
set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 100.0)
@variable(model, 50.0 <= z_rotational_radius <= 100.0)
@variable(model, 70.0 <= track_lever_length <= 100.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

@variable(model, θx)
@variable(model, θz)



register(model, :objective, 6, objective, autodiff=true)
register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)

################## objective function ##################
@NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))


################## constraints        ##################
#hier ligt der Fehler
@NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1 )


set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)

JuMP.optimize!(model)

solution = [value(x_rotational_radius);value(z_rotational_radius);value(track_lever_length);value(tie_rod_length)]
objectiv = objective_value(model)
status = termination_status(model)


objective(1, 20, solution...)


checkConstraints°(solution...)


checkConstraints°((96.0, 134.0, 146.0, 235.0)...)


objective(1, 20, (96.0, 134.0, 146.0, 235.0)...)