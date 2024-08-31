"""
    rotate3(A,n,alpha,shift=[0;0;0])

calculates the Rotation matirx RM an rotation point R.

#Arguments
- `A`: Point to be rotated
- `n`: vector around which to rotate
- `alpha`: the angle to be rotated
- `shift=[0;0;0]`: used to rotate around axes that are not going through the origin

#Returns
-`RM`: Rotation matrix
-`R`: rotated point 
"""
function rotate3(A,n,alpha,shift=[0;0;0])
    # the point A is rotated (counter clockwise) by alpha degrees around vector n. 
    # shift has to be used to rotate around axes that are not going through the origin

    n = n/norm°(n) # n = n/norm(n)

    # generate rotation matrix
    RM_1 = [(n[1]^2)*(1-cos(alpha))+cos(alpha);
        n[2]*n[1]*(1-cos(alpha))+n[3]*sin(alpha);
        n[3]*n[1]*(1-cos(alpha))-n[2]*sin(alpha)]
    
    RM_2 = [n[1]*n[2]*(1-cos(alpha))-n[3]*sin(alpha);
        (n[2]^2)*(1-cos(alpha))+cos(alpha);
        n[3]*n[2]*(1-cos(alpha))+n[1]*sin(alpha)]
    
    RM_3 = [n[1]*n[3]*(1-cos(alpha))+n[2]*sin(alpha);
        n[2]*n[3]*(1-cos(alpha))-n[1]*sin(alpha);
        (n[3]^2)*(1-cos(alpha))+cos(alpha)]
    
    RM = [RM_1 RM_2 RM_3]

    if shift ==[0;0;0]
        R = RM*A
    else
        A = A-shift
        R = RM*A
        R = R+shift
    end
    #returning rotation matrix RM and rotated point R
    return RM,R
end
"""
    applyMatrixRotation(vec::SVector{3,T}, basis1::Matrix{T}, basis2::Matrix{T}) where {T}

    performs a rotation of a vector vec using two sets of three orthogonal unit vectors basis1 and basis2.



"""
function applyMatrixRotation(vec::AbstractVector{T1}, basis1::Matrix{T2}, basis2::Matrix{T3}) where{T1,T2,T3}
    if length(vec) != 3
        throw(ArgumentError("Length of AbstractVec does not match size of SVector"))
    end
    vec =  SVector{3, T1}(vec)

    u1 = SVector{3, T2}(basis1[:,1])
    v1 = SVector{3, T2}(basis1[:,2])
    w1 = SVector{3, T2}(basis1[:,3])
    u2 = SVector{3, T3}(basis2[:,1])
    v2 = SVector{3, T3}(basis2[:,2])
    w2 = SVector{3, T3}(basis2[:,3])

    return applyMatrixRotation(vec,u1,v1,w1,u2,v2,w2)
end

function applyMatrixRotation(vec::SVector{3,T1}, u1::SVector{3,T2},v1::SVector{3,T2},w1::SVector{3,T2},u2::SVector{3,T3},v2::SVector{3,T3},w2::SVector{3,T3}) where{T1,T2,T3}
    R = ([dot(u1, u2) dot(u1, v2) dot(u1, w2);
        dot(v1, u2) dot(v1, v2) dot(v1, w2);
        dot(w1, u2) dot(w1, v2) dot(w1, w2)])

    return R * vec

end

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

function EulerRotationMatrix(alpha,beta,gamma,v)

    Rx = [1.0 0.0 0.0; 0.0 cos(alpha) -sin(alpha); 0.0 sin(alpha) cos(alpha)]
    Ry = [cos(beta) 0.0 sin(beta); 0.0 1.0 0.0; -sin(beta) 0.0 cos(beta)]
    Rz = [cos(gamma) -sin(gamma) 0.0; sin(gamma) cos(gamma) 0.0; 0.0 0.0 1.0]
    R = Rx * Ry * Rz
    v = R*v
end
