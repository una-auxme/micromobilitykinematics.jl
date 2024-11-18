
"""
    exportXML(vehicle::Vehicle)

"""
function exportXML(vehicle::Vehicle; fileName = "", path = @__DIR__ )

end



function paramtoXML(doc, )


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


"""

TODO
"""
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
    function load_data(θ::Tuple, path = @__DIR__)

    

"""
function load_data(θ::Tuple, path = @__DIR__)
    θx,θz = θ
    data_path = joinpath(path, "optimization\\data\\data($θx,n)\\opt_series($θx,$θz).jld2")
    @load data_path opt_series
    return opt_series
end
