s1 = Suspension(30.0)

s2 = Steering(53.0, 151.0, 169.0, 249.0)

s2.kinematics!(20.0,40.0,s2,s1)


m = Measurements()

m.wheel_base


s2.track_lever_mounting_points_ucs


angle_δo(s2)

s2.circle_joints

s2.circle_joints_neutral

s2.base_vec_wheel_ucs

s2.sphere_joints
s2.sphere_joints_neutral

a = (s2.rotational_component.x_rotational_radius,s2.rotational_component.z_rotational_radius, s2.track_lever.length, s2.tie_rod.length)







s1 = Suspension(30.0)

s2 = Steering(53.0, 151.0, 169.0, 249.0)






steeringkinematicsMOVED!((5.0,3.0),s2,s1)
steeringkinematicsNEUTRAL!((0,0),s2,s1)

s2.track_lever_mounting_points_ucs


m = Measurements(c, s2)


steering_objective((30,20),m, s2, s1 )


steering = Steering(50.0, 123.0, 227.0, 129.0)
suspension = Suspension(30.0)
steering.kinematics!((10.0,20.0),steering,suspension)
steering.wheel_ucs_position

suspension = Suspension(30.0)

chassi = Chassi()
measurements = Measurements(chassi, steering)


step_size = 1




θx_max, θz_max = (10, 20)

checkConstraints(step_size, (θx_max, θz_max), steering, suspension, measurements)



steering = Steering(50.0, 123.0, 227.0, 129.0)
suspension = Suspension(30.0)
step_size = 1
θx_max, θz_max = (10, 20)

θ_tuples = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
steerings = [kinematicsUNTILmount°(θ_tuple, steering, suspension) for θ_tuple in θ_tuples]
kin_Bool = []
angle_Bool = []
sin_Bool = []
angle_Bool2 = []
track_Bool = []
steerings[1,2].track_lever_mounting_points_ucs


steerings[end, end]

for steering in steerings
    kinematicsASOFmount°!(steering)
    update°!(steering)
end

for steering in steerings 
    θx, θz = (steering.θx, steering.θz)
    if steering.θz == θz_max
        continue
    end
    steering_next = steerings[θx+1, θz+2]
    @show steering_next.δo
    push!(sin_Bool, outer_sigularity_constraint(steering, steering_next))


end 

steering
measurments = Measurements(Chassi(),steerings[1, end])
push!(track_Bool, TrackingCircleConstraint(steerings[1, end], measurments))

nothing !== findfirst(x -> x ==false, track_Bool)











steering = Steering(79.0, 112.0, 138.0, 216.0)
suspension = Suspension(30.0)
suspensionkinematics!(suspension)
step_size = 1
θx_max,θz_max = (10,35)
suspension


kin_Bool = []
angle_Bool = []
sin_Bool = []
track_Bool = []

# One-time calculation of the kinematics and storage in instances until the intersection needs to be checked.  
θ_tuples = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
steerings = [kinematicsUNTILmount°(θ_tuple, steering, suspension) for θ_tuple in θ_tuples]

# checks if steering.sphere_joints is calculable (intersection is posible)
for steering in steerings
    push!(kin_Bool, KinematicDependence(steering))
    update°!(steering)
end

# calculation of complete kinematics necessary (steering.sphere_joints)
for steering in steerings
    #for (θx,θz) = (0,0) AngleDependence and SingularityConstraintis not expressive
    if steering.θx == 0 && steering.θz == 0 
        continue
    end
    push!(angle_Bool, AngleDependence(steering))

    #for (θx,θz) = (n,θz_max) SingularityConstraint checks out of bounds
    if steering.θz == θz_max
        continue
    end
    θx, θz = (steering.θx, steering.θz)
    steering_next = steerings[θx+1, θz+2]
    push!(sin_Bool, SingularityConstraint(steering,steering_next))

end


steerings[1, 35].δo
# Is the turning circle maintained in the planar plane?
measurments = Measurements(Chassi(),steerings[1, end])
push!(track_Bool, TrackingCircleConstraint(steerings[1, end], measurments))



@show kin_Bool 
@show angle_Bool
@show sin_Bool 
@show track_Bool



getValue(steering)

steering = Steering(50.0, 123.0, 227.0, 129.0)
suspension = Suspension(30.0)
steering.kinematics!((10.0,20.0),steering,suspension)
steering.wheel_ucs_position

suspension = Suspension(30.0)

chassi = Chassi()
measurements = Measurements(chassi, steering)


step_size = 1




θx_max, θz_max = (10, 25)

checkConstraints(step_size, (θx_max, θz_max), steering, suspension, measurements)








δo = 15
sind(δo)
r_is = 1000 / sind(24) 









@show sin_Bool


@show steerings[1,2].circle_joints



lower_bourder = (50.0,100.0, 100.0, 100.0)
upper_bourder = (100.0, 140.0,150.0,270.0)
max_angleConfig = (10,35)

random_search(upper_bourder,lower_bourder, max_angleConfig)
















for steering in steerings

    push!(kin_Bool, KinematicDependence(steering))

    θx, θz = (steering.θx, steering.θz)

    update°!(steering)
    #for (θx,θz) = (0,0) angleDependence is not posible
    if steering.θx == 0 && steering.θz == 0 
        continue
    end
    push!(angle_Bool, angle_dependence°(steering))

    #for (θx,θz) = (n,θz) SingularityConstraint is out of bounds
    if steering.θz == θz_max
        continue
    end
    steering_next = steerings[θx+1, θz+2]
    push!(sin_Bool, SingularityConstraint(steering,steering_next))
end

@show steerings[1,2].circle_joints

steerings[1,3].θx


@show kin_Bool 
@show angle_Bool 
@show sin_Bool 

########## Testing if structs are usable for JuMP.jl ###############


mutable struct Test2

    x::Union{<:Real, VariableRef}
    y::Union{<:Real, VariableRef}
    function Test2(x::T, y::T) where {T <: Union{Real, VariableRef}}
        inst = new()
        inst.x = x
        inst.y = y 
        return inst
    end
end


function objective(test::Test2)
    3*test.x+4*test.y 
end


# creating model 
model = Model(optimizer_with_attributes(Ipopt.Optimizer,"tol" => 1e3,  "max_iter" => 600))

set_optimizer_attribute(model, "print_level", 5)


@variable(model, 0 <= x )
@variable(model, 0 <= y )

@constraint(model, 8 <= 2x + y)
@constraint(model, 10 <= x + 2y)
@constraint(model, x + y  <= 15 )

t = Test2(x,y)
@objective(model, Min, objective(t))



optimize!(model)

value(x)
value(y)
objectiv = objective_value(model)
status = termination_status(model)



Plane([1,3,4], [3,4,2])

GeoSpatialRelations.abstractvec_to_svector(["d","d","d"])


Plane()

























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


# Speichern
@save "myclass.jld2" obj b

# Laden
@load "myclass.jld2" obj b

b 

obj.x


println(obj)


   

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
260.0
)
max_angleConfig = (10,35)

@thread

random_search(upper_bourder, lower_bourder, max_angleConfig)
optim((1,20), upper_bourder, lower_bourder, max_angleConfig)

grid_optim(upper_bourder, lower_bourder, max_angleConfig)

sol_dict = optim_series(6, (1,20), upper_bourder, lower_bourder, max_angleConfig)

steering = sol_dict[1].steering


suspension = Suspension(30.0)
    chassi = Chassi()

    steering_objective((1,35),chassi,st, suspension)

param = steering.rotational_component.x_rotational_radius,steering.rotational_component.z_rotational_radius, steering.track_lever.length, steering.tie_rod.length 

objective°(10,20,round(param[1]),round(param[2]),round(param[3]),round(param[4]))


opt_seires_times_6 = sol_dict

pathTOdata = joinpath(@__DIR__,"optimization\\data\\data(0,n)\\opt_series(0,23).jld2")
@save pathTOdata opt_seires_times_6

@load pathTOdata opt_series

plot_optda_series(opt_series)



opt_series[9].status



sol = optim_IN_LOOP((1,20), upper_bourder, lower_bourder, max_angleConfig)

optda = optim((1,20), upper_bourder, lower_bourder, max_angleConfig)

optda = optim((1,20), upper_bourder, lower_bourder, max_angleConfig, param = (106.0, 146.0, 188.0, 200.0))





optda.objective


pathTOdata = joinpath(@__DIR__,"optimization\\data\\backup\\current_obj.jld2")
@load pathTOdata steering obj

obj
pathTOdata = joinpath(@__DIR__,"optimization\\data\\backup\\best_obj(1.0, 20.0).jld2")
@load pathTOdata data


steering, objective = data["(1.0, 20.0)"]

steering 

objective

optda.input

get_insights(optda.steering)

optda.objective


objective(1,20,optda.input...)

pathTOdata = joinpath(@__DIR__,"optimization\\data\\optda.jld2")
@__DIR__

@save pathTOdata optda


@load pathTOdata obj

@load pathTOdata steering

obj 

steering.θz


optda.status

s = [1]

s = hcat(s, [1])



objective(1,20,99.0, 138.0, 179.0, 195.0)



checkConstraints°(125.8391621794173, 122.33674484085564, 138.92104776793516, 224.38308699244647)


right_circcirc_min_intersec_dependence°(10,35, 125.8391621794173, 122.33674484085564, 138.92104776793516, 224.38308699244647)




# Anzahl der Iterationen
n = 10

# Dictionary für die Ergebnisse
result_dict = Dict{Int, Float64}()

# for-Schleife, um die Berechnungen auszuführen und im Dictionary zu speichern
for i in 1:n
    # Beispielberechnung: i^2 + 2*i + 1
    result = i^2 + 2 * i + 1
    # Ergebnis in das Dictionary speichern
    result_dict[i] = result
end

result_dict



