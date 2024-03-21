using Pkg
using LinearAlgebra
# u2 = [1; 0; 0]
# v2 = [0; 1; 0]
# w2 = [0; 0; 1]

# u1 = [0.166531; -0.0324328; 0.985503]
# v1 = [0.372557; 0.927443; -0.0324328]
# w1 = [-0.912945; 0.372557; 0.166531]



"""
    EulerRotationAngles(u1,v1,w1,u2,v2,w2,sequence)

calculates the Euler angles of rotation

# Arguments
- `u1`: base vector Coordinate System 1
- `v1`: base vector Coordinate System 1
- `w1`: base vector Coordinate System 1
- `u2`: base vector Coordinate System 2
- `v2`: base vector Coordinate System 2
- `w2`: base vector Coordinate System 2
- `sequence`: vector as a sequencs of (x,y,z) to be rotated from System 1 to System 2


# Returns
- `alpha, beta, gamma`: Rotation angles
"""
function EulerRotationAngles(u1,v1,w1,u2,v2,w2,sequence)
#Rotation Matrix to rotate from Coordinate System 1 with u1,v1,w1 as base vectors to Coordinate System 2 with base vectors u2,v2,w2
R = ([dot(u1, u2) dot(u1, v2) dot(u1, w2);
        dot(v1, u2) dot(v1, v2) dot(v1, w2);
        dot(w1, u2) dot(w1, v2) dot(w1, w2)])

if sequence == 1 #Sequence x-y-z
    beta = 0
    alpha = 0
    gamma = 0
elseif sequence == 2 #Sequence z-y-x
beta = -rad2deg(-asin(R[1,3]))        
alpha = -atand(R[2,3],R[3,3])
gamma = -atand(R[1,2],R[1,1])
else
    alpha = 0
    beta = 0
    gamma = 0
end

return alpha, beta, gamma
end


function EulerRotationAngles(u1,v1,w1,u2,v2,w2; konvention = "z-y-x")
#Rotation Matrix to rotate from Coordinate System 1 with u1,v1,w1 as base vectors to Coordinate System 2 with base vectors u2,v2,w2
R = ([dot(u1, u2) dot(u1, v2) dot(u1, w2);
        dot(v1, u2) dot(v1, v2) dot(v1, w2);
        dot(w1, u2) dot(w1, v2) dot(w1, w2)])

if konvention == "z-y-x" 
    θ = rad2deg(-asin(R[1,3]))  #um y-Achse
    ϕ = rad2deg(asin((R[2,3])/(cos(-asin(R[1,3]))))) #um x-Achse
    ψ = rad2deg(asin((R[1,2])/(cos(-asin(R[1,3]))))) #um z-Achse
end

return ψ, θ, ϕ
end


function EulerRotationMatrix(angle; rot = "Rx")

    if rot == "Rx"
        Rx = [1.0 0.0 0.0; 0.0 cos(angle) -sin(angle); 0.0 sin(angle) cos(angle)]
        return Rx
    end
    if rot == "Ry"
        Ry = [cos(angle) 0.0 sin(angle); 0.0 1.0 0.0; -sin(angle) 0.0 cos(angle)]
        return Ry
    end
    if rot == "Rz"
        Rz = [cos(angle) -sin(angle) 0.0; sin(angle) cos(angle) 0.0; 0.0 0.0 1.0]
        return Rz
    end
    nothing
end

"""
    EulerExtrinsicRotations(alpha, beta, gamma, v;xkonvention = "zxz")

TBW
"""
function EulerBodyfixedINRoomfixed(psi, theta, phi, v;xkonvention = "z-y-x") 

    if xkonvention == "z-y-x"
        R = EulerRotationMatrix(psi; rot = "Rz") * EulerRotationMatrix(theta; rot = "Ry") * EulerRotationMatrix(phi; rot = "Rx") 
        return R * v
    end
    nothing
end

#=
function EulerExtrinsicRotations(v,args...;xkonvention = "zxz",alpha = 0.0, beta = 0.0, gamma=0.0,) 
    
    alpha, beta, gamma = EulerRotationAngles(args...)

    v = EulerExtrinsicRotations(alpha,beta,gamma,v,xkonvention)

    v
end
=#
function EulerExtrinsicRotations(v, u1,v1,w1,u2,v2,w2; tran = false)
    R = ([dot(u1, u2) dot(u1, v2) dot(u1, w2);
        dot(v1, u2) dot(v1, v2) dot(v1, w2);
        dot(w1, u2) dot(w1, v2) dot(w1, w2)])
    if tran == false
        return R * v
    elseif tran == true
        return R' * v
    end
    nothing
end



function EulerRotationMatrix(alpha,beta,gamma,v)

    Rx = [1.0 0.0 0.0; 0.0 cos(alpha) -sin(alpha); 0.0 sin(alpha) cos(alpha)]
    Ry = [cos(beta) 0.0 sin(beta); 0.0 1.0 0.0; -sin(beta) 0.0 cos(beta)]
    Rz = [cos(gamma) -sin(gamma) 0.0; sin(gamma) cos(gamma) 0.0; 0.0 0.0 1.0]
    R = Rx * Ry * Rz
    v = R*v
end
# phi,theta,psi = EulerRotationAngles(base1,u1,v1,w1,base2,u2,v2,w2)
