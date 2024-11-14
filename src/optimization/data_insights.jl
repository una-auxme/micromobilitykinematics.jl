"""



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
    save(steering::Steering, objective::T) where {T<:Number}

    saves the current best objective of the optimaization

#Arguments
-`steering::Steering`: Instance of a specific steering
-`objective`: objective value of the current Iteration

#Returns
-`nothing`
"""
function save_current_best_objective°(steering::Steering, objective::T) where {T<:Number}

    pathTOdata = joinpath(@__DIR__,"data\\backup\\current_obj.jld2")
    try    
        @load pathTOdata obj
        if objective < obj
            @show obj = objective
            @save pathTOdata steering obj
        end
    catch
        @show obj = objective
        @save pathTOdata steering obj
    end
end



function save_best_objective()
    pathTOdata = joinpath(@__DIR__,"data\\backup\\current_obj.jld2")
    @load pathTOdata steering obj

    # get values
    θx = steering.θx
    θz = steering.θz
    objective = obj

    pathTOdata2 = joinpath(@__DIR__,"data\\backup\\best_obj($θx, $θz).jld2")

    try
        @load pathTOdata2 data
        push!(data["($θx, $θz)"], (steering, objective))
        @save pathTOdata2 data
    catch err
        data = Dict()
        data["($θx, $θz)"] = (steering, objective)
        @save pathTOdata2 data
    end
    rm(pathTOdata)
end


"""


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
    plt_param = plot(title = "components lenghts ", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))
    plt_objective = plot(title = "objective", xlabel = "set ID", ylabel = "[mm]", legend = true, background_color_legend = RGBA(255,255,255, .01))  
    plt_δo = plot(title = "turning angle", xlabel = "set ID", ylabel = "[°]", legend = true, background_color_legend = RGBA(255,255,255, .01))

    
    scatter!(plt_objective, keys, objective)
    scatter!(plt_δo , keys, δo)

    repeat_keys = [[i for _ in 1:length(param[i])] for i in 1:length(param)]
    scatter!(plt_param,repeat_keys,param )
    plt_merge = plot(plt_objective,plt_δo,plt_param ,layout = (3, 1),size = (500, 900))


    display(plt_merge)
end

