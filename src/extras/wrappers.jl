include("GeoSpatialRelations/src/GeoSpatialRelations.jl")
using .GeoSpatialRelations


"""
    intersection(steering, args...)

calculating the intersection point for a circle and a sphere

# Arguments
- `steering::Steering`: instance of a steering
- `args...`

# Returns
There are three possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.
- There are two intersection then `intersection_point1::Vector` and `intersection_point2::Vector` are returned.


"""
function intersection(steering::Steering, args...)
    try
         GeoSpatialRelations.intersection(args...)
    catch err
        bt = catch_backtrace()
        capture_error!(steering, err, bt)
        rethrow()
    end
end



"""
    intersection(suspension, args...)

calculating the intersection point for a circle and a sphere

# Arguments
- `suspension::Suspension`: instance of a suspension
- `args...`

# Returns
There are three possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.
- There are two intersection then `intersection_point1::Vector` and `intersection_point2::Vector` are returned.


"""
function intersection(suspension::Suspension, args...)
    try
         GeoSpatialRelations.intersection(args...)
    catch err
        bt = catch_backtrace()
        capture_error!(suspension, err, bt)
        rethrow()
    end
end