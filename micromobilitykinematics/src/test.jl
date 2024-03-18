
 include("circcirc.jl")
 include("circsphere.jl")
 include("core.jl")
 include("suspensionkinematics.jl")
 


s = Suspension()


w = WheelMount()


s.damper.upper_fixture
s.damper.length
s.lowerwishbone.rotation_axis
s.lowerwishbone.rotation_axis[1]
s.kinematics!(s.lowerwishbone,s.upperwishbone,s.damper,w)