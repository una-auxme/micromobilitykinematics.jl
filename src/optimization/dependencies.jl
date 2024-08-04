"""
angleDependence(steering::Steering)

checks angle dependence

#Arguments 
- `steering::Steering`: Instance of a specific steering

#Returns:
-`::Bool`
"""
function angleDependence(arge...)
try
    if !(angle_dependence(arge...) > 0)
        error("dependence violation:    steering angle ")
    end
catch err
    #println(stacktrace(err))
    return false 
end 
return true 
end 

"""
    KinematicDependence(steering::Steering)

checks kinematic dependence

#Arguments
-`steering::Steering`: Instance of a specific steerin

#Returns:
-`::Bool`
"""
function KinematicDependence(arge...)
    try
        if !(left_circsphere_plane_dependence(arge...)  <= 0)
            error("dependence violation:    no kinematic connection between circsphere and plane ")       
        elseif !(right_circsphere_plane_dependence(arge...) <= 0)
            error("dependence violation:    no kinematic connection between circsphere and plan")
        elseif !(left_circcirc_min_intersec_dependence(arge...) <= 0)
            error("dependence violation:    no kinematic connection between circ and circ -> Component to short")
        elseif !(right_circcirc_min_intersec_dependence(arge...) <= 0)
            error("dependence violation:    no kinematic connection between circ and circ -> Component to short")
        elseif !(left_circcirc_max_intersec_dependence(arge...) <= 0)
            error("dependence violation:    no kinematic connection between circ and circ -> Component to long")
        elseif !(right_circcirc_max_intersec_dependence(arge...) <= 0)
            error("dependence violation:    no kinematic connection between circ and circ -> Component to long")
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
        if !(outer_sigularity_constraint(arge...) < 0)
            error("constraint violation:    parameters stept over singularitay ") 
        elseif !(inner_sigularity_constraint(arge...) < 0)
            error("constraint violation:    parameters stept over singularitay ") 
        end
    catch err
        #println(stacktrace(err))
        return false
    end
    return true 
end

"""
    TrackingcircleConstraint(steering::Steering, measurments::Measurements)

    checks tracking circle constraint

#Arguments
-`steering::Steering`: Instance of a specific steering
-`measurments::Measurements`: Instance of a specific all relevant Measurements of the vehicle
    
#Returns:
-`::Bool`
"""
function TrackingCircleConstraint(arge...)
    try 
        if !(track_circle_dependence(arge...) >= 0)
            error("tracking circle dependence violation:    vehicle coundn't drive min tracking circle ") 
        end
    catch err
        #println(stacktrace(err))
        return false
    end
    return true 
end

"""
checkConstraints(step_size,max_angelConfig,compLength,radius)

    checks  all constraints and dependences

#Arguments 
-`step_size`: step_size in which the angular area should be checked
-`max_angelConfig`: tuple (θx, θz)
-`compLength`: tuple (xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength)
-`radius`: desired track circle radius

#Returns:
-`::Bool`
"""
function checkConstraints(step_size::T, max_angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension, measurments::Measurements; info = false) where {T<:Integer}
    θx_max, θz_max = max_angleConfig

    try
        θ_tuple = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
        if  θx_max == 0      
            kin_Bool = [KinematicDependence(θ_tuple[1,j],steering, suspension)  for j in 1:step_size:(θz_max+1)]
            angle_Bool = [angleDependence(θ_tuple[1,j+1],steering,suspension)  for j in 1:step_size:(θz_max)]
            sin_Bool = [SingularityConstraint(θ_tuple[1,j+1],steering,suspension)  for j in 1:step_size:(θz_max)]
            track_Bool = TrackingCircleConstraint(θ_tuple[1,(θz_max +1)],steering, suspension, measurments)

            if info
                print("\n_____________Info_____________\n")
                print("\nBoolean Matrix of kinematic dempendence:\n")
                print("$kin_Bool\n")
                print("Boolean Matrix of angle dempendence:\n")
                print("$angle_Bool\n")
                print("Boolean Matrix of singularity constraint:\n")
                print("$sin_Bool\n")
                print("tracking circle constraint:\n")
                print("$track_Bool\n")
            end

            if nothing !== findfirst(x -> x ==false, kin_Bool)
              error("kinematic dempendence couldn't be matched by parameters ")
            elseif nothing !== findfirst(x -> x ==false, angle_Bool)
              error("angle dempendence couldn't be matched by parameters ")
            elseif nothing !== findfirst(x -> x ==false, sin_Bool)
              error("singularity constraint couldn't be matched by parameters ")
            elseif !(track_Bool)
              error("tracking circle constraint couldn't be matched by parameters ")
            end
        else 
            kin_Bool = [KinematicDependence(θ_tuple[i,j],steering,suspension) for i in 1:step_size:(θx_max+1), j in 1:step_size:(θz_max+1)]

            angle_Bool = [angleDependence(θ_tuple[1,j+1],steering,suspension)  for j in 1:step_size:(θz_max)]
            angle_Bool2 = [angleDependence(θ_tuple[i,j+1],steering,suspension)  for i in step_size:step_size:(θx_max+1), j in 1:step_size:(θz_max)]

            sin_Bool = [SingularityConstraint(θ_tuple[1,j+1],steering,suspension)  for j in 1:step_size:(θz_max)]
            sin_Bool2 = [SingularityConstraint(θ_tuple[i,j+1],steering,suspension)  for i in step_size:step_size:(θx_max+1), j in 1:step_size:(θz_max)]

            track_Bool = TrackingCircleConstraint(θ_tuple[1,(θz_max +1)],steering, suspension, measurments)

            if info 
                print("\n_____________Info_____________\n")
                print("\nBoolean Matrix of kinematic dempendence:\n")
                print("$kin_Bool\n")
                print("Boolean Matrix of angle dempendence:\n")
                print("$angle_Bool\n")
                print("$angle_Bool2\n")
                print("Boolean Matrix of singularity constraint:\n")
                print("$sin_Bool\n")
                print("$sin_Bool2\n")
                print("tracking circle constraint:\n")
                print("$track_Bool\n")
            end

            if nothing !== findfirst(x -> x ==false, kin_Bool) 
                error("kinematic dempendence couldn't be matched by parameters ")
              elseif nothing !== findfirst(x -> x ==false, angle_Bool) || nothing !== findfirst(x -> x ==false, angle_Bool2)
                error("angle dempendence couldn't be matched by parameters ")
              elseif nothing !== findfirst(x -> x ==false, sin_Bool) || nothing !== findfirst(x -> x ==false, sin_Bool2)
                error("singularity constraint couldn't be matched by parameters ")
              elseif !(track_Bool)
                error("tracking circle constraint couldn't be matched by parameters ")
              end
        end
    catch err
        if info 
            println(err)
        end
        #println(stacktrace(err))
        return false
    end
    return true
end

"""
    random_search(upper_bourder::Tuple{Float64, Float64, Float64, Float64},lower_bourder::Tuple{Float64, Float64, Float64, Float64},max_angleConfig; info = false, radius = 3500, step_size = 1 )

random search with given bourder for the parameters and given angular area for rotary component 

#Arguments:
-`upper_bourder::Tuple{Float64, Float64, Float64, Float64}`: upper bourder Tuple (xSteeringRadius, zSteeringRadius, TrackLeverLength, TieRodLength)
-`lower_bourder::Tuple{Float64, Float64, Float64, Float64}`: lower bourder Tuple (xSteeringRadius, zSteeringRadius, TrackLeverLength, TieRodLength)
-`max_angleConfig`: maximal angular area for rotary component (defult: (0,35))

#Keywords:

-`info::Bool`: true if info should be printed
-`radius`: desired track circle radius (defult: 3500)
-`step_size`: step_size in which the angular area should be checked

#Returns:
-`compLength`: tuple (xSteeringRadius, zSteeringRadius, TieRodLength, TrackLeverLength)
"""
function random_search(upper_bourder::Tuple{T,T,T,T},lower_bourder::Tuple{T,T,T,T}, max_angleConfig::Tuple{I,I} ; info = false, step_size = 1 ) where {T<:Real, I<:Integer}
    b = true 
    i = 0
    while b
        compLength =[rand(l:u) for (l,u) in zip(lower_bourder, upper_bourder)]
        if info 
            print("\n\n\n------| Iteration $i |-------\n") 
            print("Parmeters: $compLength")
        end

        suspension = Suspension(30)
        steering = Steering(compLength...)
        measurments = Measurements()

        if checkConstraints(step_size, max_angleConfig,steering,suspension, measurments; info = true) == true
            return tuple(compLength...)
        end
        i +=1
    end
    return compLength
end
