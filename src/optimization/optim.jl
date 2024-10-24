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
    θx_, θz_ = θ
    start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = getValue(steering)

    #selection of the used solver
    model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e0, 
                                                        "acceptable_tol" => 1e0, 
                                                        "dual_inf_tol" =>1e0, 
                                                        "compl_inf_tol"=>  1e0,
                                                        "constr_viol_tol"=> 1e0,
                                                        "max_iter" => 600))
    #set_optimizer_attribute(model, "algorithm", :LD_MMA)


    # define the parameters to be optimised
    @variable(model, 50.0 <= x_rotational_radius <= 200.0)
    @variable(model, 50.0 <= z_rotational_radius <= 200.0)
    @variable(model, 70.0 <= track_lever_length <= 200.0)
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



function optim(θ::Tuple{T,T},vec_steerings::Vector{Steering}) where {T<:Number}
    b = true
    lk = ReentrantLock()
    @threats for steering in vec_steerings
        try 
            model = create_model(θ, steering)
            optimize!(model)
            objectiv,solution,status = get_model_solution(model)

            
            lock(lk) do 
                

            end

        catch


        end
    end

end 




####################### Versuche 1






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






mkdir()





solution = (value(x_rotational_radius),value(z_rotational_radius),value(track_lever_length),value(tie_rod_length))
objectiv = objective_value(model)
status = termination_status(model)





objective(10,20,100.36,
118.99,
135.30,
224.23)



    steering = Steering(79.72433062896792, 122.04062852415808, 134.14620864478988, 229.13168306790027)
    suspension = Suspension(30.0)
    chassi = Chassi()
    suspensionkinematics!(suspension)


    update!((1,10),steering, suspension)







    @save pathTOdata steering



    @load "steering.jld2" steering 

    print(steering)
    
    
    s
    
    struct MyClass
        x::Int
        y::String
    end

    obj = MyClass(42, "Hallo Welt")

# Speichern
@save "myclass.jld2" obj

# Laden
@load "myclass.jld2" obj

println(obj)


    pathTOdata = joinpath(@__DIR__,"data\\steering.jld2")

    CSV.write(pathTOdata, df, delim=`\t`)


    θx_max, θz_max  = (10,35)
    angleConfig = (θx_max, θz_max)

    steering = Steering(79.72433062896792, 122.04062852415808, 134.14620864478988, 229.13168306790027)
    
    suspension = Suspension(30)
    suspensionkinematics!(suspension)
    checkConstraints(1,angleConfig,steering,suspension) ? 1.0 : 0.0

    objective(0,1,79.72433062896792, 122.04062852415808, 134.14620864478988, 229.13168306790027)




    checkConstraints°(96.0, 134.0, 146.0, 235.0)


    s = kinematicsUNTILmount°((0,1), steering, suspension)

    kinematicsASOFmount°!(s)

    update°!(s)


    s.circle_joints

 

    θx_max, θz_max  = (10,35)
    angleConfig = (θx_max, θz_max)

    steering = Steering(79.72433062896792, 122.04062852415808, 134.14620864478988, 229.13168306790027)
    
    suspension = Suspension(30)
    suspensionkinematics!(suspension)


        kin_Bool = []
        angle_Bool = []
        sin_Bool = []
        track_Bool = []
        step_size  = 1
        θx_max, θz_max = (10,35)
        # One-time calculation of the kinematics and storage in instances until the intersection needs to be checked.  
        θ_tuples = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
        steerings = [kinematicsUNTILmount°(θ_tuple, steering, suspension) for θ_tuple in θ_tuples]

        # checks if steering.sphere_joints is calculable (intersection is posible)


        i = 1
        steerings[1,1].θz 
        steerings[1,1].track_lever_mounting_points_ucs

        update°!(steerings[1,2])
    	
        

        steerings[1,2].circle_joints



        for steering in steerings
            push!(kin_Bool, KinematicDependence(steerings[i]))
            
            if steering.θx == 0 && steering.θz == 0 
                println("θx: $(steering.θx) | θz: $(steering.θz)")
                
            else 
                update°!(steering)
            end 
        end


        steerings[1,2].circle_joints



optimize!(model)
if !is_solved_and_feasible(model)
    error("Solver did not find an optimal solution")
end

solution = [value(x_rotational_radius);value(z_rotational_radius);value(track_lever_length);value(tie_rod_length)]
objectiv = objective_value(model)
status = termination_status(model)


objective(1, 20, solution...)


checkConstraints°(solution...)


checkConstraints°((96.0, 134.0, 146.0, 235.0)...)


objective(1, 20, (96.0, 134.0, 146.0, 235.0)...)







lower_bourder = (50.0,
50.0, 
70.0, 
195.0)
upper_bourder = (100.0,
200.0,
200.0,
200.0
)
max_angleConfig = (10,35)

random_search(upper_bourder, lower_bourder, max_angleConfig)

objective(1,20,99.0, 138.0, 179.0, 195.0)



checkConstraints°(125.8391621794173, 122.33674484085564, 138.92104776793516, 224.38308699244647)


right_circcirc_min_intersec_dependence°(10,35, 125.8391621794173, 122.33674484085564, 138.92104776793516, 224.38308699244647)