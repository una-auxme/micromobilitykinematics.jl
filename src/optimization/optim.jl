"""
    create_sphere_model(θ::Tuple{T,T})

    creats model for optimization with sphere dependences

    #Arguments
    -`θ::Tuple{T,T}`: tuple of Angles (θx, θz)
    -`start_set`: Parameter start values ["x_rotational_radius";"z_rotational_radius";"track_lever_length";"tie_rod_length"]

    #Returns:
    -`model`: returns a jump model

    See Also
    """
function create_model(θ::Tuple{T,T}, steering::Steering) where {T<:Number}
    start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = getValue(steering)

    #selection of the used solver
    model = Model(NLopt.Optimizer)
    set_optimizer_attribute(model, "algorithm", :LD_MMA)


    # define the parameters to be optimised
    @variable(model, 50.0 <= x_rotational_radius <= 100.0)
    @variable(model, 50.0 <= z_rotational_radius <= 100.0)
    @variable(model, 70.0 <= track_lever_length <= 100.0)
    @variable(model, 195.0 <= tie_rod_length <= 260.0)

    # fixed variables used in optimisation
    @variable(model, θx)
    @variable(model, θz)


    # used and unknown functions must be registered for optimisation
    register(model, :objective, 6, objective, autodiff=true)
    register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)

    # define the objectie of the given NL-Problem
    @NLobjective(model, Min, objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))
    
    # define the constraint of the given NL-Problem
    @NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1 )

    # initialisation of the parameters
    set_start_value(x_rotational_radius, start_x_rotational_radius)
    set_start_value(z_rotational_radius, start_z_rotational_radius)
    set_start_value(track_lever_length, start_track_lever_length)
    set_start_value(tie_rod_length, start_tie_rod_length)

    fix(θx, θx_)
    fix(θz,θz_)

    return model
end

"""
    get_model_solution(model)

    gets the importend informations of the model solution

#Arguments:
-`model`: jump model

#Reruns:
-`objectiv`: value of the final objectiv
"""
function get_model_solution(model)
    x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length, ~ = all_variables(model)
    solution = [value(x_rotational_radius);value(z_rotational_radius);value(track_lever_length);value(tie_rod_length)]
    objectiv = objective_value(model)
    status = termination_status(model)
    return objectiv,solution,status
end 


function save()



end



function optim()

    solution =["x_rotational_radius";"z_rotational_radius";"track_lever_length";"tie_rod_length"]
    start_val = ["start x_rotational_radius";"start z_rotational_radius";"start track_lever_length";"start tie_rod_length"]
    distance_error = ["error"]
    ratio = ["ratio [w/L]";"ratio [%]"]
    δ = ["δi";"δo"]
    sol_status = ["solution status"]
    radius = ["radius"]








end 

func









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