using LinearAlgebra


function lineplane(support_point_line::Vector{Float64}, direction::Vector{Float64}, normal::Vector{Float64},support_point_plane::Vector{Float64})

    x = dot(normal, support_point_plane .- support_point_line) / dot(normal, direction)

    intersection_point = support_point_line .+ x * direction

    return intersection_point

end