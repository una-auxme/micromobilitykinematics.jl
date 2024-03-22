using LinearAlgebra
using Rotations

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
    n = n/norm(n)

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