
using LinearAlgebra




"""
    circcirc(pos_circ, r_circ, n_circ, pos_circ_2, r_circ_2)

calculates the intersecting points of two circles in one plane in three dimensional space  

# Arguments
- pos_circ: position of cirlce 1
- r_circ: radius of circle 1
- pos_circ_2: position of circle 2
- r_circ_2: radius of circle 2
- n_circ: normal vector of both circles

# Returns
- `p1`: intersecting point
- `p2`: intersecting point
"""
function circcirc(pos_circ, r_circ, n_circ, pos_circ_2, r_circ_2)
    n_circ = n_circ/norm(n_circ)        #n_circ must be a unit vector
    d = norm(pos_circ -pos_circ_2)                 
    if d > (abs(r_circ) + abs(r_circ_2)) || ((abs(d) + abs(r_circ)) <  abs(r_circ_2)) || ((abs(d) + abs(r_circ_2)) <  abs(r_circ))   # Check if circles do not intersect      
        error("circles do not intersect")
    elseif d == (abs(r_circ) + abs(r_circ_2)) ||   (abs(d) + abs(r_circ)) == abs(r_circ_2) # Check if circles are tangent at one point     
        p1 = pos_circ + (pos_circ -pos_circ_2)/norm(pos_circ -pos_circ_2) * r_circ;
        p2 = [];
    else                                        
        lambda = acos((norm(pos_circ -pos_circ_2)^2 + r_circ^2-r_circ_2^2)/(2*r_circ*(norm(pos_circ -pos_circ_2))));
        h = sin(lambda)*r_circ;  
        a = sqrt(r_circ^2-h^2);
        ps = pos_circ - ((pos_circ-pos_circ_2)/norm(pos_circ-pos_circ_2))*a;
        v = cross(n_circ,(pos_circ-pos_circ_2)/norm(pos_circ-pos_circ_2));
        p1 = ps - h*(v/norm(v));
        p2 = ps + h*(v/norm(v));
        return p1,p2
    end
end

