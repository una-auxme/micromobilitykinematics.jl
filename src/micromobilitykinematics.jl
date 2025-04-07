module micromobilitykinematics
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
    using .GeoSpatialRelations
    using GLMakie

    
    using JuMP
    using Ipopt
    using NLopt


    export steeringkinematics!, steeringkinematics, steeringkinematicsNEUTRAL!, steeringkinematicsMOVED!
    export suspensionkinematics!, suspensionkinematicsNEUTRAL!, suspensionkinematicsMOVED!
    export update!
    export angle_δi!, angle_δi, angle_δo!, angle_δo
    export random_search

    export exportXML

    export optim_series, optim, grid_optim
    export checkConstraints, checkConstraints°, steering_objective, objective°

    export plot_optda_series, plot_optda_gird_δ, plot_optda_gird_obj

    
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
    include("utils/io.jl")
end
