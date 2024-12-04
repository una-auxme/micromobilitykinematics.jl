"""
AngleDependency(steering::Steering)

checks angle dependency

#Arguments 
- `steering::Steering`: Instance of a specific steering

#Returns:
-`::Bool`
"""
function AngleDependency(arge...)
    try
        if !(angle_dependency(arge...) > 0)
            error("dependency violation:    steering angle ")
        end
    catch err
        #println(stacktrace(err))
        return false 
    end 
    return true 
end 

"""
    KinematicDependency(steering::Steering)

checks kinematic dependency

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns:
-`::Bool`
"""
function KinematicDependency(arge...)
    try
        if !(left_circsphere_plane_dependency(arge...)  <= 0)
            error("dependency violation:    no kinematic connection between circsphere and plane ")       
        elseif !(right_circsphere_plane_dependency(arge...) <= 0)
            error("dependency violation:    no kinematic connection between circsphere and plan")
        elseif !(left_circcirc_min_intersec_dependency(arge...) <= 0)
            error("dependency violation:    no kinematic connection between circ and circ -> Component to short")
        elseif !(right_circcirc_min_intersec_dependency(arge...) <= 0)
            error("dependency violation:    no kinematic connection between circ and circ -> Component to short")
        elseif !(left_circcirc_max_intersec_dependency(arge...) <= 0)
            error("dependency violation:    no kinematic connection between circ and circ -> Component to long")
        elseif !(right_circcirc_max_intersec_dependency(arge...) <= 0)
            error("dependency violation:    no kinematic connection between circ and circ -> Component to long")
        end
    catch err
        #println(err)
        #println(stacktrace(err))
        return false 
    end
    return true 
end

"""
    SingularityConstraint(steering::Steering, suspension::Suspension)

checks singularity constraints

#Arguments
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

#Returns:
-`::Bool`
"""
function SingularityConstraint(arge...)
    try 
        if !(outer_singularity_constraint(arge...) < 0)
            error("constraint violation:    parameters stepped over singularitay ") 
        elseif !(inner_singularity_constraint(arge...) < 0)
            error("constraint violation:    parameters stepped over singularitay ") 
        end
    catch err
        #println(stacktrace(err))
        return false
    end
    return true 
end

"""
    TrackingcircleConstraint(steering::Steering, measurements::Measurements)

    checks tracking circle constraint

#Arguments
-`steering::Steering`: Instance of a specific steering
-`measurements::Measurements`: Instance of a specific all relevant Measurements of the vehicle
    
#Returns:
-`::Bool`
"""
function TrackingCircleConstraint(arge...)
    try 
        if !(track_circle_Dependency(arge...) >= 0)
            error("tracking circle dependency violation:    vehicle couldn't drive min tracking circle ") 
        end
    catch err
        #println(stacktrace(err))
        return false
    end
    return true 
end

"""
checkConstraints(step_size, max_angleConfig::Tuple, steering::Steering, suspension::Suspension)

    checks  all constraints and dependencies

#Arguments 
-`step_size`: step_size in which the angular area should be checked
-`maxangleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        -`θx`: maximal Angle of rotation of the rotation component around the x-axis
        -`θz`: maximal Angle of rotation of the rotation component around the z-axis
-`steering::Steering`: Instance of a specific steering
-`suspension::Suspension`: Instance of a specific suspension

#Returns:
-`::Bool`:
        -`false`: It is not possible to match the constraints in a satisfactory manner
        -`true`: It is possible to match the constraints in a satisfactory manner
"""
function checkConstraints(step_size, max_angleConfig::Tuple, steering::Steering, suspension::Suspension) 
    θx_max, θz_max = max_angleConfig
    try
        kin_Bool = []
        angle_Bool = []
        sin_Bool = []
        track_Bool = []

        # One-time calculation of the kinematics and storage in instances until the intersection needs to be checked.  
        θ_tuples = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
        steerings = [kinematicsUNTILmount°(θ_tuple, steering, suspension) for θ_tuple in θ_tuples]
        # checks if steering.sphere_joints is calculable (intersection is posible)
        for steering in steerings
            push!(kin_Bool, KinematicDependency(steering))
        end

        if nothing !== findfirst(x -> x ==false, kin_Bool)
            error("Thread $(Threads.threadid()):> kinematic dependency couldn't be matched by parameters ")
        end


        # Calculate the further kinematics for all angular positions of the steering system.
        for steering in steerings
            if steering.θx == 0 && steering.θz == 0 
                continue
            else 
                update°!(steering)
            end 
        end

        # calculation of complete kinematics necessary (steering.sphere_joints)
        for steering in steerings
            #for (θx,θz) = (0,0) AngleDependency and SingularityConstraintis not expressive
            if steering.θx == 0 && steering.θz == 0 
                continue
            end
            push!(angle_Bool, AngleDependency(steering))

            # for (θx,θz) = (n,θz_max) SingularityConstraint checks out of bounds
            if steering.θz == θz_max
                continue
            end
            θx, θz = (steering.θx, steering.θz)
            steering_next = steerings[θx+1, θz+2]
          	
            push!(sin_Bool, SingularityConstraint(steering,steering_next))
        end

        # Is the turning circle maintained in the planar plane?
        measurements = Measurements(Chassis(),steerings[1, end])
        push!(track_Bool, TrackingCircleConstraint(steerings[1, end], measurements))

        if nothing !== findfirst(x -> x ==false, angle_Bool)
            error("Thread $(Threads.threadid()):> angle dependency couldn't be matched by parameters ")
          elseif nothing !== findfirst(x -> x ==false, sin_Bool)
            error("Thread $(Threads.threadid()):> singularity constraint couldn't be matched by parameters ")
          elseif nothing !== findfirst(x -> x ==false, track_Bool)
            error("Thread $(Threads.threadid()):> tracking circle constraint couldn't be matched by parameters ")
        end
    catch err
        #println("\n\n\n $(err.stack) \n\n\n")
        println("$err")
        return false
    end
    return true
end

"""
    checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

The wrapper function of the checkConstraints procedure is employed for the purposes of optimisation.
! function°(): symbolises that this function should only be used within the optimisation !

#Arguments
-`x_rotational_radius`: Length of the rotational component around the x-axis. (detailed info in doc)
-`z_rotational_radius`: Length of the rotational component around the z-axis. (detailed info in doc)
-`track_lever_length`: Length of the track lever. (detailed info in doc)
-`tie_rod_length`: Length of tie rod. (detailed info in doc)

#Returns
-`binary`:
        -`0`: It is not possible to match the constraints in a satisfactory manner.
        -`1`: It is possible to match the constraints in a satisfactory manner.

"""
function checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    θx_max, θz_max  = (10,35)

    println("Thread $(Threads.threadid()):> checkConstraints°")

    if typeof(x_rotational_radius) == Float64
        println("Thread $(Threads.threadid()):> ($(x_rotational_radius), $(z_rotational_radius), $(track_lever_length), $(tie_rod_length))")
    end

    angleConfig = (θx_max, θz_max)

    steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    
    suspension = Suspension(30)
    suspensionkinematics!(suspension)
    #println(":> $(checkConstraints(1,angleConfig,steering,suspension) ? 1.0 : 0.0)")
    return checkConstraints(1,angleConfig,steering,suspension) ? 1.0 : 0.0
end



"""
    random_search(upper_border::Tuple{Float64, Float64, Float64, Float64},lower_border::Tuple{Float64, Float64, Float64, Float64},max_angleConfig; info = false, radius = 3500, step_size = 1 )

random search with given border for the parameters and given angular area for rotary component 

#Arguments:
-`upper_border::Tuple{Float64, Float64, Float64, Float64}`: upper border Tuple (x_rotational_radius, z_rotational_radius, track_lever.length, tie_rod.length)  (guidline = (100.0, 140.0,150.0,270.0))
-`lower_border::Tuple{Float64, Float64, Float64, Float64}`: lower border Tuple (x_rotational_radius, z_rotational_radius, track_lever.length, tie_rod.length) (guidline = (50.0,100.0, 100.0, 100.0))
-`max_angleConfig`: maximal angular area for rotary component (defult: (0,35))

#Keywords:

-`info::Bool`: true if info should be printed
-`radius`: desired track circle radius (defult: 3500)
-`step_size`: step_size in which the angular area should be checked

#Returns:
-`compLength`: tuple (x_rotational_radius, z_rotational_radius, track_lever.length, tie_rod.length)
"""
function random_search(upper_border::Tuple{T,T,T,T},lower_border::Tuple{T,T,T,T}, max_angleConfig::Tuple{I,I} ; info = false, step_size = 1 ) where {T<:Real, I<:Integer}
    param = nothing
    valid_param = false

    i = 0
    while !valid_param
        param = [rand(l:u) for (l,u) in zip(lower_border, upper_border)]

        if info 
            println("Thread $(Threads.threadid()):> Random Search Iteration $i") 
            println("Thread $(Threads.threadid()):> Parameters: $param \n")
        end

        suspension = Suspension(30)
        suspensionkinematics!(suspension)
        steering = Steering(param...)

        valid_param = checkConstraints(step_size, max_angleConfig,steering,suspension)
        i +=1
    end
    return Tuple(param)
end
