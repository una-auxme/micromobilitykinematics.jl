include("core.jl")
include("suspensionkinematics.jl")
include("steeringkinematics.jl")
include("EulerRotation.jl") 
include("rotate3.jl")
include("calc_basis_vectors.jl")



using Intersect3D


lineplane()


s = Suspension()


Intersect3D.

w = WheelMount()



s.kinematics!(s)



s2 = Steering(53.0, 151.0, 169.0, 249.0)





s2.kinematics!(20.0,40.0,s2,s)


function test(t::Array{Vector{Float64},1})
    t    
end


t = [1.0; 2.0]

typeof(t)


t = [[1,0,0],[1,2,3]]



include("rotate3.jl")

using Rotations


rotate3()

RotationVec()