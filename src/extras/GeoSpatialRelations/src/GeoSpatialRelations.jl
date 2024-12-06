module GeoSpatialRelations

using LinearAlgebra
using StaticArrays


export intersection
export Circle, Plane, Line, Sphere

include("core/core.jl")
include("core/_convert.jl")
include("func/intersection.jl")

end # module GeoSpatialRelations