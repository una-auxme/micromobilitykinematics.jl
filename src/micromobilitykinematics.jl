module micromobilitykinematics
    using Documenter    
    using LinearAlgebra
    using StaticArrays
    using DataFrames
    using JLD2
    using Base.Threads
    using Plots;gr()
    #using PlotlyJS
    using LightXML
    #using GeoSpatialRelations
    include("GeoSpatialRelations/src/GeoSpatialRelations.jl")
    using .GeoSpatialRelations  
    #using Colors  # (oder: using ColorTypes)

    
    using JuMP
    using Ipopt
    using NLopt


    export steeringkinematics!, steeringkinematics, steeringkinematicsNEUTRAL!, steeringkinematicsMOVED!
    export suspensionkinematics!, suspensionkinematicsNEUTRAL!, suspensionkinematicsMOVED!
    export update!
    export angle_δi!, angle_δi, angle_δo!, angle_δo
    export random_search

    export exportXML

    export create_model_for_pose, get_model_solution, optim_at_pose, optim_series_at_pose, grid_optim, create_model_for_range, optim_over_range
    export checkConstraints, checkConstraints°, ackermann_deviation, ackermann_deviation_for_pose, ackermann_deviation_over_range

    export plot_optda_series, plot_optda_gird_δ, plot_optda_gird_obj

    export lounch_gui





    include("core/types.jl")

    include("extras/calc_basis_vectors.jl")
    include("extras/rotations.jl")
    include("extras/wrappers.jl")

    include("core/kinematics/steering.jl")
    include("core/kinematics/suspension.jl")
    include("core/on_instance.jl")
    include("core/error_handling.jl")

    include("optimization/dependencies/core_dependencies.jl")
    include("optimization/dependencies/functions_for_dependencies.jl")
    include("optimization/dependencies/dependencies.jl")
    include("optimization/objective.jl")
    include("optimization/driver.jl")

    include("utils/insights.jl")
    include("utils/io.jl")
    #include("utils/plots.jl")

    include("extension_hooks/gui_stub.jl")

end
