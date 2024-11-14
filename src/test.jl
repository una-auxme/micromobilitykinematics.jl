###################### Versuche 1

θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (54.0, 157.0, 189.0, 196.0) #(69.0, 144.0, 180.0, 200.0) (106.0, 146.0, 188.0, 200.0)# (67.0, 158.0, 195.0, 196.0)
θx_,θz_ = (10,20)
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e0, 
                                                        "acceptable_tol" => 1e0, 
                                                        "dual_inf_tol" =>1e0, 
                                                        "compl_inf_tol"=>  1e0,
                                                        "constr_viol_tol"=> 1e0,
                                                        "max_iter" => 600))
                                                        
#model = Model(NLopt.Optimizer)
#verwendung von :LD_MMA funktionier besser
# lokales Minimum bereschnen da wir mit vor random suche bereits ein guten Start wert ermitteln und dannach diesen minimieren.
# jedes werte paar hat vermutlich seinen eigenen besten wert.

#set_optimizer_attribute(model, "algorithm", :LD_MMA)
#set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 200.0)
@variable(model, 50.0 <= z_rotational_radius <= 200.0)
@variable(model, 70.0 <= track_lever_length <= 200.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

@variable(model, θx)
@variable(model, θz)


steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

register(model, :objective, 6, objective, autodiff=true)
register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)


################## objective function ##################
@NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))

################## constraints        ##################
#hier ligt der Fehler
@NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1)


set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)

JuMP.optimize!(model)


####################### Versuche 2






θx_max, θz_max  = (10,35)
start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = (106.0, 146.0, 188.0, 200.0) #(96.0, 134.0, 146.0, 235.0)
θx_,θz_ = (1,20)
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e-2, 
                                                        "acceptable_tol" => 1e-2, 
                                                        "dual_inf_tol" =>1e-2, 
                                                        "compl_inf_tol"=>  1e-2,
                                                        "constr_viol_tol"=> 1e-2,
                                                        "max_iter" => 600))
                                                        
#model = Model(NLopt.Optimizer)
#verwendung von :LD_MMA funktionier besser
# lokales Minimum bereschnen da wir mit vor random suche bereits ein guten Start wert ermitteln und dannach diesen minimieren.
# jedes werte paar hat vermutlich seinen eigenen besten wert.

#set_optimizer_attribute(model, "algorithm", :LD_MMA)
#set_optimizer_attribute(model, "print_level", 5)


@variable(model, 50.0 <= x_rotational_radius <= 200.0)
@variable(model, 50.0 <= z_rotational_radius <= 200.0)
@variable(model, 70.0 <= track_lever_length <= 200.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)

@variable(model, θx)
@variable(model, θz)


steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

register(model, :objective, 6, objective, autodiff=true)


register(model, :left_circsphere_plane_dependence°, 6, left_circsphere_plane_dependence°, autodiff=true)
register(model, :right_circsphere_plane_dependence°, 6, right_circsphere_plane_dependence°, autodiff=true)
register(model, :left_circcirc_min_intersec_dependence°, 6, left_circcirc_min_intersec_dependence°, autodiff=true)
register(model, :right_circcirc_min_intersec_dependence°, 6, right_circcirc_min_intersec_dependence°, autodiff=true)
register(model, :left_circcirc_max_intersec_dependence°, 6, left_circcirc_max_intersec_dependence°, autodiff=true)
register(model, :right_circcirc_max_intersec_dependence°, 6, right_circcirc_max_intersec_dependence°, autodiff=true)
register(model, :outer_sigularity_constraint°, 6, outer_sigularity_constraint°, autodiff=true)
register(model, :inner_sigularity_constraint°, 6, inner_sigularity_constraint°, autodiff=true)
register(model, :track_circle_dependence°, 6, track_circle_dependence°, autodiff=true)
register(model, :angle_dependence°, 6, angle_dependence°, autodiff=true)


step_size = 1
################## objective function ##################
@NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))

################## constraints        ##################
#hier ligt der Fehler

@NLconstraint(model, C1_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circsphere_plane_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C1_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circsphere_plane_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C2_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circcirc_min_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C2_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circcirc_min_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C3_1[i=0:step_size:θx_max, j=0:step_size:θz_max],  left_circcirc_max_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)
@NLconstraint(model, C3_2[i=0:step_size:θx_max, j=0:step_size:θz_max],  right_circcirc_max_intersec_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0)

@NLconstraint(model, C4_1[i=step_size:step_size:θx_max, j=0:step_size:(θz_max-2)], outer_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01 ) 
@NLconstraint(model, C4_2[i=0, j=step_size:step_size:(θz_max-2)], outer_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01) 

@NLconstraint(model, C5_1[i=step_size:step_size:θx_max, j=0:step_size:(θz_max-2)], inner_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01 ) 
@NLconstraint(model, C5_2[i=0, j=step_size:step_size:(θz_max-2)], inner_sigularity_constraint°(i, j, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) <= 0.01) 

@NLconstraint(model, C6_1[i=step_size:step_size:θx_max, j=0:step_size:θz_max], angle_dependence°(i, j,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 0.01 )
@NLconstraint(model, C6_2[i=0, j=step_size:step_size:θz_max],  angle_dependence°(θx, θz,x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 0.01 )



@NLconstraint(model, C7, track_circle_dependence°(0, θz_max, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)  >= 0)



set_start_value(x_rotational_radius, start_x_rotational_radius)
set_start_value(z_rotational_radius, start_z_rotational_radius)
set_start_value(track_lever_length, start_track_lever_length)
set_start_value(tie_rod_length, start_tie_rod_length)

fix(θx, θx_)
fix(θz,θz_)

JuMP.optimize!(model)






lower_bourder = (50.0,
50.0, 
70.0, 
195.0)
upper_bourder = (100.0,
200.0,
200.0,
260.0
)
max_angleConfig = (10,35)

@thread

random_search(upper_bourder, lower_bourder, max_angleConfig)
optim((1,20), upper_bourder, lower_bourder, max_angleConfig)

grid_optim(upper_bourder, lower_bourder, max_angleConfig)

sol_dict = optim_series(6, (1,20), upper_bourder, lower_bourder, max_angleConfig)

optda = sol_dict[6]

optda.objective








sol_dict