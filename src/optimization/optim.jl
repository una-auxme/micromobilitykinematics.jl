

mutable struct Test2

    x::Union{<:Real, VariableRef}
    y::Union{<:Real, VariableRef}
    function Test2(x::Union{<:Real, VariableRef}, y::Union{<:Real, VariableRef})
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


@variable(model, 70.0 <= track_lever_length <= 100.0)
@variable(model, 195.0 <= tie_rod_length <= 260.0)
@variable(model, 50.0 <= x_rotational_radius <= 100.0)
@variable(model, 50.0 <= z_rotational_radius <= 100.0)

@variable(model, θx)
@variable(model, θz)

@variable(model, penalty)


suspension = Suspension(30.0)



steering = Steering(value(x_rotational_radius), value(z_rotational_radius), value(track_lever_length), value(tie_rod_length))

chassi = Chassi()



register(model, :steering_objective, 6, steering_objective, autodiff=true)

ref::VariableRef

################## objective function ##################
@NLobjective(model, Min, steering_objective(θx, θz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))







