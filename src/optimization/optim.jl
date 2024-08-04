
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e3,  "max_iter" => 600))

set_optimizer_attribute(model, "print_level", 5)


@variable(model, 70.0 <= track_lever_length <= 100.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)
@variable(model, 50.0 <= x_rotational_radius <= 100.0)
@variable(model, 50.0 <= z_rotational_radius <= 100.0)

@variable(model, θx)
@variable(model, θz)

@variable(model, penalty)


register(model, :objective, 6, objective, autodiff=true)



################## objective function ##################
@objective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))


################## constraints        ##################
@constarint(model, C1[i=0, j=step_size:step_size:θz_max], angle_dependence()  )

@constraint(model, C2_1[i=0, j=0:step_size:θz_max],  Lcircsphere_plane_dependence(i, j, x_rotational_radius, z_rotational_radius, TieRodLength, TrackLeverLength) <= 0)
@constraint(model, C3_1[i=0, j=0:step_size:θz_max],  Rcircsphere_plane_dependence(i, j,xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength) <= 0)
@constraint(model, C4_1[i=0, j=0:step_size:θz_max],  Lcirccirc_min_intersec_dependence(i, j,xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength) <= 0)
@constraint(model, C5_1[i=0, j=0:step_size:θz_max],  Rcirccirc_min_intersec_dependence(i, j,xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength) <= 0)
@constraint(model, C6_1[i=0, j=0:step_size:θz_max],  Lcirccirc_max_intersec_dependence(i, j,xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength) <= 0)
@constraint(model, C7_1[i=0, j=0:step_size:θz_max],  Rcirccirc_max_intersec_dependence(i, j,xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength) <= 0)



