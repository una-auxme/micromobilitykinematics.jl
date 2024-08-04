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

c = Chassi()

s2.kinematics!((20.0,40.0),s2,s1)
s2.kinematics!((10.0,20.0),s2,s1)

s2.track_lever_mounting_points_ucs


m = Measurements(c, s2)


steering_objective((30,20),m, s2, s1 )



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