module micromobilitykinematic

    using Documenter
    using LinearAlgebra
    using GeoSpatialRelations
    using StaticArrays


    include("core/core.jl")
    include("core/suspensionkinematics.jl")
    include("core/steeringkinematics.jl")
    include("custom_funcs/rotations.jl") 
    include("custom_funcs/calc_basis_vectors.jl")
    include("core/functions_on_instance.jl")
    include("optimization/core_dependencies.jl")    
end
