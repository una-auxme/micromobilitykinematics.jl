include("core.jl")
include("suspensionkinematics.jl")
include("steeringkinematics.jl")
include("custom_funcs/rotations.jl") 
include("custom_funcs/calc_basis_vectors.jl")





s1 = Suspension()


s2 = Steering(53.0, 151.0, 169.0, 249.0)

s2.kinematics!(20.0,40.0,s2,s1)

