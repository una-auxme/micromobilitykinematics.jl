module micromobilitykinematic

    using Documenter
    using LinearAlgebra
    using GeoSpatialRelations
    using StaticArrays


    include("core.jl")
    include("suspensionkinematics.jl")
    include("steeringkinematics.jl")
    include("custom_funcs/rotations.jl") 
    include("custom_funcs/calc_basis_vectors.jl")
    
end
