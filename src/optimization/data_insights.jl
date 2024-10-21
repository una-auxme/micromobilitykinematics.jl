
"""



"""
function get_insights(steering::Steering)
    # Extrahiere die benötigten Werte aus dem Steering-Objekt
    θx = steering.θx
    θz = steering.θz
    δi = steering.δi
    δo = steering.δo

    x_rotational_radius = steering.rotational_component.x_rotational_radius
    z_rotational_radius = steering.rotational_component.z_rotational_radius

    track_lever_length = steering.track_lever.length
    tie_rod_length = steering.tie_rod.length

    # Erstelle eine Tabelle (DataFrame)
    df = DataFrame(
        Parameter = ["θx", "θz", "δi", "δo", "x_rotational_radius", "z_rotational_radius", "track_lever.length", "tie_rod.length"],
        Value = [θx, θz, δi, δo, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length]
    )

    return df
end 