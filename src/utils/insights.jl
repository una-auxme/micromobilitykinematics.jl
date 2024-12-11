
"""
    get_insights(steering::Steering)


"""
function get_insights(steering::Steering)
    # Extrahiere die benötigten Werte aus dem Steering-Objekt
    θx = steering.θx
    θz = steering.θz
    δi = steering.δi
    δo = steering.δo


    x_rotational_radius = steering.rotational_component.x_rotational_radius
    z_rotational_radius = steering.rotational_component.z_rotational_radius

    track_lever_length = steering.track_lever.length
    tie_rod_length = steering.tie_rod.length

    # Erstelle eine Tabelle (DataFrame)
    df = DataFrame(
        Parameter = ["θx", "θz", "δi", "δo", "x_rotational_radius", "z_rotational_radius", "track_lever.length", "tie_rod.length"],
        Value = [θx, θz, δi, δo, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length]
    )

    return df
end 



function get_components_lengths(steering::Steering)

    x_rotational_radius = steering.rotational_component.x_rotational_radius
    z_rotational_radius = steering.rotational_component.z_rotational_radius

    track_lever_length = steering.track_lever.length
    tie_rod_length = steering.tie_rod.length

    return x_rotational_radius,z_rotational_radius,track_lever_length,tie_rod_length

end



"""
    grid_data_obj(θ_max::Tuple{I,I}, steering::Steering, suspension::Suspension, chassi::Chassis; step_size = 1) where {I <: Any}

calculates the objective of the given 

"""
function grid_data_obj(θ_max::Tuple{Any,Any}, steering::Steering, suspension::Suspension, chassi::Chassis; step_size = 1) 
    θx_max, θz_max = θ_max
    
    
    θ_tuple = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    objective = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]

    for θ in θ_tuple
        if θ == (0,0)
            continue
        end
        θx, θz  = θ
        objective[θx+1,θz+1] = steering_objective(((θx, θz)), chassi, steering, suspension)

    end

    return objective
end


"""
    grid_data_δ(θ_max::Tuple{I,I}, steering::Steering; step_size = 1) where {I <: Any}

Calculates the turning angles of the vehicle for each turning angles of the rotation component (θx, θz) of the steering system.

#Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.

#Keywords
-`step_size`: The step size of the grid

#Retruns
-`δi, δo`: Tuple of turning angles Matrix 


"""
function grid_data_δ(θ_max::Tuple{Any,Any}, steering::Steering; step_size = 1) 
    θx_max, θz_max = θ_max
    
    
    θ_tuple = [(i, j) for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    δi  = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]
    δo  = [ 0.0 for i in 0:step_size:θx_max, j in 0:step_size:θz_max]

    for θ in θ_tuple
        if θ == (0,0)
            continue
        end
        θx, θz  = θ
        temp_steering = update(((θx, θz)), steering, suspension)
        δi[θx+1,θz+1] = temp_steering.δi 
        δo[θx+1,θz+1] = temp_steering.δo 
    end
    return δi, δo
end

"""
    plot_optda_series(optda_dict::Dict{Int64,Any})
    

"""
function plot_optda_series(optda_dict::Dict{Int64,Any})

    #
    keys = []
    param = []
    objective = []
    δo = []


    for (key,optda) in optda_dict 
        push!(δo,optda.steering.δo)
        push!(objective,optda.objective)
        push!(param, [get_components_lengths(optda.steering)...])
        push!(keys,key)
    end 

    # 
    println(keys)
    plt_param = Plots.plot(title = "components lengths ", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))
    plt_objective = Plots.plot(title = "objective", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))  
    plt_δo = Plots.plot(title = "turning angle", xlabel = "set ID", ylabel = "[°]", legend = true, background_color_legend = RGBA(255,255,255, .01))

    
    Plots.scatter!(plt_objective, keys, objective)
    Plots.scatter!(plt_δo , keys, δo)

    repeat_keys = [[i for _ in 1:length(param[i])] for i in 1:length(param)]
    Plots.scatter!(plt_param,repeat_keys,param )
    plt_merge = Plots.plot(plt_objective,plt_δo,plt_param ,layout = (3, 1),size = (500, 900))


    display(plt_merge)
    nothing
end


"""
    plot_optda_gird_obj(θ_max::Tuple{I,I}, steering::Steering, suspension::Suspension, chassi::Chassis) where {I <: Any}

Create a plot in which the objective (Ackermann ratio) are mapped to the turning angles of the rotation component (θx, θz) of the steering system.   

#Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.
-`suspension::Suspension`: Instance of a specific suspension in which the kinematics were previously calculated.
-`chassi::Chassis`: Instance of a Chassis

#Returns

"""
function plot_optda_gird_obj(args...)
    θx_max, θz_max = args[1]
    θx = 0:1:θx_max
    θz = 0:1:θz_max

    objective = grid_data_obj(args...)

    sur_obj = PlotlyJS.surface(;x=θx, y=θz, z=objective)
    #title="Lenkabweichung in Abhängigkeit der Stellwinkel (θx, θz) der Rotationskomponente der Lenkgeometrie"
    layout = Layout(autosize=true, margin=attr(l=10, r=10, b=10, t=10),
    scene=attr(
        xaxis=attr(title="θx in [°]"),
        yaxis=attr(title="θz in [°]"),
        zaxis=attr(title="objective in [mm]"),
        scene = attr(aspectmode="cube") 
    ),
    width=600,
    height=600,
    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4)
    )

    PlotlyJS.plot(sur_obj, layout)
end

"""
    plot_optda_gird_δi(θ_max::Tuple{I,I}, steering::Steering) where {T <: Any}
    	
Create a plot in which the turning angles of the vehicle are mapped to the turning angles of the rotation component (θx, θz) of the steering system.

#Arguments
-`θ_max::Tuple{I,I}`: maximal turning angles of the rotation component
-`steering::Steering`: Instance of a specific steering mechanism in which the kinematics were previously calculated.

#Returns


"""
function plot_optda_gird_δ(args...)
    θx_max, θz_max = args[1]
    θx = 0:1:θx_max
    θz = 0:1:θz_max


    δi, δo  = grid_data_δ(args...)

    sur_δi = PlotlyJS.surface(;x=θx, y=θz, z=δi)
    sur_δo = PlotlyJS.surface(;x=θx, y=θz, z=δo)

    #title="Lenkabweichung in Abhängigkeit der Stellwinkel (θx, θz) der Rotationskomponente der Lenkgeometrie"
    layout = Layout(autosize=true, margin=attr(l=10, r=10, b=10, t=10),
    scene=attr(
        xaxis=attr(title="θx in [°]"),
        yaxis=attr(title="θz in [°]"),
        zaxis=attr(title="objective in [mm]"),
        scene = attr(aspectmode="cube") 
    ),
    width=600,
    height=600,
    scene_camera_eye=attr(x=2.0, y=1.4, z=1.4)
    )

    PlotlyJS.plot([sur_δi, sur_δo], layout)
end


