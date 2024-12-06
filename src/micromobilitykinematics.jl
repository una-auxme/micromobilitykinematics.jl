module micromobilitykinematic
    using Documenter    
    using LinearAlgebra
    using StaticArrays
    using DataFrames
    using JLD2
    using Base.Threads
    using Plots;gr()
    using PlotlyJS
    using LightXML
    include("extras/GeoSpatialRelations/src/GeoSpatialRelations.jl")
    #using .GeoSpatialRelations

    using JuMP
    using Ipopt
    using NLopt

    
    include("core/core.jl")
    include("core/suspensionkinematics.jl")
    include("core/steeringkinematics.jl")
    include("extras/rotations.jl") 
    include("extras/calc_basis_vectors.jl")
    include("core/functions_on_instance.jl")
    include("optimization/core_dependencies.jl")    
    include("optimization/dependencies.jl")
    include("optimization/functions_for_dependencies.jl")
    include("optimization/objective.jl")
    include("utils/insights.jl")
    include("optimization/optim.jl")
    include("utils/insights.jl")    
end
