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
step_size = 1
θx_max,θz_max = (10,35)



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