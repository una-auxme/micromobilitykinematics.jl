abstract type AbstractSpatial end


abstract type AbstractLine <: AbstractSpatial end 
abstract type AbstractPlane <: AbstractSpatial end 


for (type, supertype) in zip([:Line, :Plane], [:AbstractLine, :AbstractPlane])

    if type == :Line
        vector = :direction
    elseif type == :Plane
        vector = :normal
    end

    @eval struct $type{N,T1,T2} <: $supertype


        point::SVector{N,T1}
        $vector::SVector{N,T2}


        function $type(point::SVector{N,T1}, vector::SVector{N,T2}) where {N,T1,T2}

            if N != 3 
                throw(ArgumentError("A spatial description is only possible in dim = 3."))
            end 
            
            new{N,T1,T2}(point, vector)
        end
    end

    @eval function $type(point::Vector, vector::Vector)
        point_static = abstractvec_to_svector(point)
        vector_static = abstractvec_to_svector(vector)
        return $type(point_static, vector_static)
    end
end


abstract type AbstractSphere <: AbstractSpatial end
abstract type AbstractCircle <: AbstractSpatial end

struct Sphere{N,T} <: AbstractSphere 

    center::SVector{N, T} 
    radius::Real

    function Sphere(center::SVector{N,T} , radius::Real ) where {N,T}
        if radius <= 0
            throw(ArgumentError("Negative radius values are not permitted."))
        end

        if N != 3 
            throw(ArgumentError("A spatial description is only possible in dim = 3."))
        end 

    new{N,T}(center,radius)
    end
end

function Sphere(point::Vector, radius::Real)
    point_static = abstractvec_to_svector(point)
    return Sphere(point_static, radius)
end

struct Circle{N,T1,T2} <: AbstractSphere 

    center::SVector{N, T1} 
    radius::Real
    normal::SVector{N, T2}

    function Circle(center::SVector{N,T1} , radius::Real, normal::SVector{N, T2}) where {N, T1, T2}
        if radius <= 0
            throw(ArgumentError("Negative radius values are not permitted."))
        end

        if N != 3 
            throw(ArgumentError("A spatial description is only possible in dim = 3."))
        end 

        new{N,T1,T2}(center, radius, normal)
    end

end



function Circle(point::Vector, radius::Real, normal::Vector)
    point_static = abstractvec_to_svector(point)
    normal_static = abstractvec_to_svector(normal)
    return Circle(point_static, radius, normal_static)
end