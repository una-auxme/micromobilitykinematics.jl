function abstractvec_to_svector(vec::AbstractVector{T}) where{T}
    if length(vec) != 3
        throw(ArgumentError("The length of AbstractVector does not match the size of SVector."))
    end
    return SVector{3, T}(vec)
end
