
"""
    exportXML(inst::Union{Steering, Suspension}; file_name="SteeringParamList", path=@__DIR__ )

All important steering/suspension kinematics parameters are transferred to an XML file.

# Arguemnts
-`inst::Union{Steering, Suspension}`: Instance of a steering/suspension/... where the `kinematic!` function has already been used.

# Keywords
-`path=@__DIR__ `: Path to the folder where the xml file will be saved.


"""
function exportXML(inst::Union{Steering, Suspension}; path=@__DIR__ )
    doc = XMLDocument()
    root = create_root(doc, "list_of_parameters")
    child = new_child(root, "parameter")

    for field in fieldnames(typeof(inst))
        value = getfield(inst, field) 
        if typeof(value) <: Tuple    
            handle_tuple(child, field, inst)
        elseif typeof(value) <: Vector
            handle_vector(child, field, inst)
        elseif typeof(value) <: AbstractSteering || typeof(value) <: AbstractSuspension || typeof(value) <: AbstractVehicle
            handle_instance(child, value)
        elseif typeof(value) <: Number
            handle_single(child, field, inst)
        end
    end


    if typeof(inst) <: AbstractSteering
        file_name = "parameter_list_of_steering"

    elseif typeof(inst) <: AbstractSuspension
        file_name = "parameter_list_of_suspension"
    else
        file_name = "parameter_list_of_vehicle"
    end 

    save_file(doc,"$(file_name).xml")
end


"""
    handle_tuple(child::XMLElement, field::Symbol, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}

Tuples that exist in the core.jl module are transferred to the XML file.

# Arguments 
-`child::XMLElement`: Child to which the properties should be added in XML. 
-`field::Symbol`: the current parameter of the instance being discussed 
-`inst::Union{<:AbstractSteering,<:AbstractSuspension}`: instance that is viewed
"""
function handle_tuple(child::XMLElement,field::Symbol,inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}
    for (side, param_index) in zip(["Left_", "Right_"], [1,2])

        value = getfield(inst, field)

        # It is necessary to differentiate between the various tuples present in the core.jl module, as they require disparate handling.         
        if typeof(value[param_index]) <: AbstractSuspension
            # In the context of the class Suspension, tuples of the form (<:AbstractSuspension, <:AbstractSuspension) have been defined and are treated as follows:
            handle_instance(child, value[param_index])
        else
            # In the context of the class Steering, tuples of the form (Vector, Vector) have been defined and are treated as follows: 
    
            for (dim, vec_index) in zip(["_x","_y","_z"],[1,2,3])
                    properties_child = new_child(child,"properties")

                    name_child = new_child(properties_child, "name")
                    name_inst = string(typeof(inst))
                    field_name = string(field)
                    name = side*name_inst*"_"*field_name*dim
                    add_text(name_child, name)

                    typeCode_child = new_child(properties_child, "typeCode")
                    add_text(typeCode_child, "mm")

                    value_child = new_child(properties_child, "value")
                    value = getfield(inst, field)[param_index][vec_index]
                    add_text(value_child, "$value mm")

                    comment_child = new_child(properties_child, "comment")
                    add_text(comment_child, "")

                    key_child = new_child(properties_child, "key")
                    add_text(key_child, "")

                    tolerance_child = new_child(properties_child, "tolerance")
                    add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")

                end
            end
    end
end




"""
    handle_vector(child::XMLElement, field::Symbol, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}


Vectors that exist in the core.jl module are transferred to the XML file.

# Arguments 
-`child::XMLElement`: Child to which the properties should be added in XML. 
-`field::Symbol`: the current parameter of the instance being discussed 
-`inst::Union{<:AbstractSteering,<:AbstractSuspension}`: instance that is viewed

"""
function handle_vector(child::XMLElement, field::Symbol, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}
    #It is unnecessary to include ID as an export parameter, as it serves no purpose in this context.
    if field == :id 
        return nothing
    end

    for (dim, vec_index) in zip(["_x","_y","_z"],[1,2,3])
        properties_child = new_child(child,"properties")

        name_child = new_child(properties_child, "name")
        # In order to differentiate between the left and right dampers or wishbones used in the suspension system, it is necessary to add the ID to the XML name. 
        if typeof(inst) <: AbstractDamper || typeof(inst) <: AbstractWishbone
            side = string(getfield(inst, :id))*"_"
        else
            side = ""
        end
        name_inst = string(typeof(inst))
        field_name = string(field)
        name = side*name_inst*"_"*field_name*dim
        add_text(name_child, name)
        
        typeCode_child = new_child(properties_child, "typeCode")
        add_text(typeCode_child, "mm")

        value_child = new_child(properties_child, "value")
        value = getfield(inst, field)[vec_index]


        add_text(value_child, "$value mm")

        comment_child = new_child(properties_child, "comment")
        add_text(comment_child, "")

        key_child = new_child(properties_child, "key")
        add_text(key_child, "")

        tolerance_child = new_child(properties_child, "tolerance")
        add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")
    end
end


"""
    handle_single(child::XMLElement, field::Symbol, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}

Single values that exist in the core.jl module are transferred to the XML file.

# Arguments 
-`child::XMLElement`: Child to which the properties should be added in XML. 
-`field::Symbol`: the current parameter of the instance being discussed 
-`inst::Union{<:AbstractSteering,<:AbstractSuspension}`: instance that is viewed

"""
function handle_single(child::XMLElement, field::Symbol, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}
    #It is unnecessary to include ID as an export parameter, as it serves no purpose in this context.
    if field == :id 
        return nothing
    end

    properties_child = new_child(child,"properties")

    name_child = new_child(properties_child, "name")
    if typeof(inst) <: AbstractDamper || typeof(inst) <: AbstractWishbone
        side = string(getfield(inst, :id))*"_"
    else
        side = ""
    end
    name_inst = string(typeof(inst))
    field_name = string(field)
    name = side*name_inst*"_"*field_name
    add_text(name_child, name)

    unit = ""
    if field == :δi || field == :δo || field == :θx || field == :θy
        unit = "deg"
    else
        unit = "mm"
    end 

    typeCode_child = new_child(properties_child, "typeCode")
    add_text(typeCode_child, unit)

    value_child = new_child(properties_child, "value")
    value = getfield(inst, field)
    add_text(value_child, "$value $unit")

    comment_child = new_child(properties_child, "comment")
    add_text(comment_child, "")

    key_child = new_child(properties_child, "key")
    add_text(key_child, "")

    tolerance_child = new_child(properties_child, "tolerance")
    add_text(tolerance_child, "Vorgabe;Nennwert;Präzise Zeichnungen")

end

"""
    handle_instance(child::XMLElement, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}

The inheritance tree of the instance is followed until each parameter of the instance is included in the XML file, thus ensuring that all relevant information is conveyed within the file.

# Arguments 
-`child::XMLElement`: Child to which the properties should be added in XML. 
-`inst::Union{<:AbstractSteering,<:AbstractSuspension}`: instance that is viewed
"""
function handle_instance(child::XMLElement, inst::Union{T,D}) where {T<:AbstractSteering, D <:AbstractSuspension}
    for field in fieldnames(typeof(inst))
        value = getfield(inst, field)
        if typeof(value) <: Tuple
            handle_tuple(child, field, inst)
        elseif typeof(value) <: Vector
            handle_vector(child, field, inst)
        elseif typeof(value) <: AbstractSteering || typeof(value) <: AbstractSuspension || typeof(value) <: AbstractVehicle
            handle_instance(child, inst)
        else 
            handle_single(child, field, inst)
        end
    end
end




"""
    save(steering::Steering, objective::T) where {T<:Number}

    saves the current best objective of the optimaization

# Arguments
-`steering::Steering`: Instance of a specific steering
-`objective`: objective value of the current Iteration

# Returns
-`nothing`
"""
function save_current_best_objective°(steering::Steering, objective::T) where {T<:Number}

    pathTOdata = joinpath(@__DIR__,"data\\backup\\current_obj.jld2")
    try    
        @load pathTOdata obj
        if objective < obj
            @show obj = objective
            @save pathTOdata steering obj
        end
    catch
        @show obj = objective
        @save pathTOdata steering obj
    end
end





































"""

TODO
"""
function save_best_objective()
    pathTOdata = joinpath(@__DIR__,"data\\backup\\current_obj.jld2")
    @load pathTOdata steering obj

    # get values
    θx = steering.θx
    θz = steering.θz
    objective = obj

    pathTOdata2 = joinpath(@__DIR__,"data\\backup\\best_obj($θx, $θz).jld2")

    try
        @load pathTOdata2 data
        push!(data["($θx, $θz)"], (steering, objective))
        @save pathTOdata2 data
    catch err
        data = Dict()
        data["($θx, $θz)"] = (steering, objective)
        @save pathTOdata2 data
    end
    rm(pathTOdata)
end



"""
    function load_data(θ::Tuple, path = @__DIR__)

    

"""
function load_data(θ::Tuple, path = @__DIR__)
    θx,θz = θ
    data_path = joinpath(path, "optimization\\data\\data($θx,n)\\opt_series($θx,$θz).jld2")
    @load data_path opt_series
    return opt_series
end
