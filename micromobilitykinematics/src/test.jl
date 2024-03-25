include("core.jl")
include("suspensionkinematics.jl")
include("steeringkinematics.jl")
include("EulerRotation.jl") 
include("rotate3.jl")
include("calc_basis_vectors.jl")



using GeoSpatialRelations

GeoSpatialRelations.Circle([1.0,0.0,0.0],20.0,[0.0,1.0,0.0])


Circle([1.0,0.0,0.0],20.0,[0.0,1.0,0.0])

lineplane()


s = Suspension()

suspensionkinematics!(s)

s

suspension = Suspension()
a = 

real_vector = convert(Vector{Real}, a )

b = 
c= 

a = convert(Vector{Real},[suspension.damper[1].upper_fixture[1]; suspension.lowerwishbone[1].bearing_rear[2]; suspension.lowerwishbone[1].bearing_rear[3]])
b = suspension.lowerwishbone[1].distance_rotation_axis_to_lower_damper_fixure
c = suspension.lowerwishbone[1].rotation_axis
GeoSpatialRelations.Circle(a,b,c)

circ_damper = GeoSpatialRelations.Circle(suspension.damper[1].upper_fixture, suspension.damper[1].length_neutral, )

suspension.damper[1].upper_fixture
suspension.damper[1].length_neutral
suspension.lowerwishbone[1].rotation_axis

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