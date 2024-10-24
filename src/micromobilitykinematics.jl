module micromobilitykinematic

    using Documenter
    using LinearAlgebra
    using GeoSpatialRelations
    using StaticArrays
    using DataFrames
    using JLD2


    using JuMP
    using Ipopt
    using NLopt

    include("core/core.jl")
    include("core/suspensionkinematics.jl")
    include("core/steeringkinematics.jl")
    include("custom_funcs/rotations.jl") 
    include("custom_funcs/calc_basis_vectors.jl")
    include("core/functions_on_instance.jl")
    include("optimization/core_dependencies.jl")    
    include("optimization/dependencies.jl")
    include("optimization/functions_for_dependencies.jl")
    include("optimization/objective.jl")
    include("optimization/functions_redefinition.jl")
    
end
