
abstract type AbstractInteractionLyt end

abstract type AbstractPlotSection <: AbstractInteractionLyt end
abstract type AbstractAngleSection <: AbstractInteractionLyt end
abstract type AbstractParamSection <: AbstractInteractionLyt end
abstract type AbstractPlotSettingsSection <: AbstractInteractionLyt end
abstract type AbstractDamperSection <: AbstractInteractionLyt end
abstract type AbstractInfoSection <: AbstractInteractionLyt end





"""
    PlotSection <: AbstractPlotSection

Ein Container zur Darstellung und Beobachtung von verschiedenen Achsen und Textkomponenten in einem Plot-Layout.  
Er enthält Felder zur Positionierung, grafischen Darstellung und Observables zur Reaktion auf interaktive Änderungen.

# Felder

- `slot`: Position im Layout-Grid.
- `ax_geom`: 3D-Achse für Geometrie.
- `ax_radii`: 2D-Achse für Radien.
- `ax_ratio`: 2D-Achse für Verhältnis-Darstellung.
- `ax_ratio_surface`: 3D-Achse für Verhältnis-Oberfläche.
- `txt_ratio_max`: Textobjekt für das maximale Verhältnis.
- `txt_ratio_min`: Textobjekt für das minimale Verhältnis.
- `obs_rotation`: Observable für Rotationszustand.
- `obs_geom_left`, `obs_geom_right`: Observables für Geometrieansichten.
- `obs_stationary`: Observable für stationäre Zustände.
- `obs_radii_θz`: Observable für Radien in θz-Richtung.
- `obs_ratio_θz`, `obs_ratio_surface`, `obs_ratio_max`, `obs_ratio_min`: Observables für Verhältnis-Daten.

# Konstruktor

- `PlotSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct PlotSection  <: AbstractPlotSection 

    #######| Grid Position
    slot::Any

    #######| Components
    ax_geom::Union{Axis3,Nothing}
    ax_radii::Union{Axis,Nothing}
    ax_ratio::Union{Axis,Nothing}
    ax_ratio_surface::Union{Axis3,Nothing}
    ax_θ_vs_δ_surface::Union{Axis3,Nothing}
    ax_deviation::Union{Axis,Nothing}
    ax_deviation_surface::Union{Axis3,Nothing}
    ax_compr_vs_δ::Union{Axis3,Nothing}

    txt_ratio_max::Union{Makie.Text,Nothing}
    txt_ratio_min::Union{Makie.Text,Nothing}
    txt_deviation_max::Union{Makie.Text,Nothing}
    txt_deviation_min::Union{Makie.Text,Nothing}

    #######| Observation

    obs_rotation::Union{Observable,Nothing}
    obs_geom_left::Union{Observable,Nothing}
    obs_geom_right::Union{Observable,Nothing}
    obs_stationary::Union{Observable,Nothing}
    obs_left_lower_wishbone_axis::Union{Observable,Nothing}
    obs_right_lower_wishbone_axis::Union{Observable,Nothing}
    obs_left_upper_wishbone_axis::Union{Observable,Nothing}
    obs_right_upper_wishbone_axis::Union{Observable,Nothing}
    obs_left_wishbone_sphere_joint::Union{Observable,Nothing}
    obs_right_wishbone_sphere_joint::Union{Observable,Nothing}

    obs_left_lower_wishbone::Union{Observable,Nothing}
    obs_left_upper_wishbone::Union{Observable,Nothing}
    obs_right_lower_wishbone::Union{Observable,Nothing}
    obs_right_upper_wishbone::Union{Observable,Nothing}

    obs_left_damper::Union{Observable,Nothing}
    obs_right_damper::Union{Observable,Nothing}

    obs_radii_θz::Union{Observable,Nothing}
    
    obs_ratio_θz::Union{Observable,Nothing}
    obs_ratio_surface::Union{Observable,Nothing}
    obs_ratio_max::Union{Observable,Nothing}
    obs_ratio_min::Union{Observable,Nothing}

    obs_θ_vs_δi_surface::Union{Observable,Nothing}
    obs_θ_vs_δo_surface::Union{Observable,Nothing}

    obs_deviation_θz::Union{Observable,Nothing}
    obs_deviation_max::Union{Observable,Nothing}
    obs_deviation_min::Union{Observable,Nothing}

    obs_deviation_surface::Union{Observable,Nothing}

    obs_compr_vs_δi::Union{Observable,Nothing}
    obs_compr_vs_δo::Union{Observable,Nothing}


    function PlotSection()
        inst = new()
        inst.slot = nothing 

        inst.ax_geom = nothing
        inst.ax_radii = nothing
        inst.ax_ratio = nothing 
        inst.ax_ratio_surface = nothing
        inst.ax_θ_vs_δ_surface = nothing
        inst.ax_deviation = nothing

        inst.txt_ratio_max = nothing 
        inst.txt_ratio_min = nothing 
        inst.txt_deviation_max = nothing
        inst.txt_deviation_min = nothing

        inst.obs_rotation = nothing
        inst.obs_geom_left = nothing
        inst.obs_geom_right = nothing
        inst.obs_stationary = nothing
        inst.obs_radii_θz = nothing
        inst.obs_ratio_θz = nothing
        inst.obs_ratio_surface = nothing
        inst.obs_ratio_max = nothing
        inst.obs_ratio_min = nothing
        inst.obs_θ_vs_δi_surface = nothing
        inst.obs_θ_vs_δo_surface = nothing
        inst.obs_deviation_θz = nothing
        inst.obs_deviation_max = nothing
        inst.obs_deviation_min = nothing

        inst.obs_deviation_surface = nothing

        inst.obs_compr_vs_δi = nothing
        inst.obs_compr_vs_δo = nothing

        return inst
    end 


end




"""
    AngleSection <: AbstractAngleSection

Ein Layout-Container zur Darstellung und Steuerung von Winkeleinstellungen innerhalb eines Plot-Layouts.  
Diese Sektion enthält GUI-Komponenten wie ein Titel-Label und ein Slider-Grid zur interaktiven Einstellung von Winkeln.

# Felder

- `slot`: Position im übergeordneten GridLayout.
- `lyt`: Inneres GridLayout für die Anordnung von Komponenten.
- `title`: Beschriftung der Sektion.
- `sg_θ`: SliderGrid zur Steuerung des Winkels θ.

# Konstruktor

- `AngleSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct AngleSection <: AbstractAngleSection # Layout Angles Section
    #######| Grid Position
    slot::Any

    #######| Components
    lyt::Union{GridLayout,Nothing}

    title::Union{Label,Nothing}

    sg_θ::Union{SliderGrid,Nothing}


    function AngleSection()
        inst = new()

        inst.slot = nothing

        inst.lyt = nothing
        inst.title = nothing
        inst.sg_θ = nothing

        return inst
    end
end


"""
    ParamSection <: AbstractParamSection

Ein GUI-Abschnitt zur Anzeige und Steuerung von Modellparametern innerhalb eines Plot-Layouts.  
Diese Sektion verwendet ein inneres GridLayout zur Organisation und bietet einen SliderGrid für interaktive Parameteranpassung.

# Felder

- `slot`: Position im äußeren Layout-Grid.
- `lyt`: Inneres GridLayout zur Anordnung der Komponenten.
- `title`: Titel oder Beschriftung des Abschnitts.
- `sg_param`: SliderGrid zur Justierung von Parametern.

# Konstruktor

- `ParamSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct ParamSection <: AbstractParamSection 
    #######| Grid Position
    slot::Any

    #######| Components
    lyt::Union{GridLayout,Nothing}
    title::Union{Label,Nothing}

    sg_param::Union{SliderGrid,Nothing}

    function ParamSection()
        inst = new()
        
        inst.slot = nothing 

        inst.lyt = nothing
        inst.title = nothing
        inst.sg_param = nothing

        return inst
    end
end

"""
    DamperSection <: AbstractDamperSection

Ein GUI-Abschnitt zur Visualisierung und Steuerung von Dämpfungsparametern, speziell für die Kompression.  
Diese Sektion nutzt ein GridLayout zur Organisation und einen SliderGrid für die Anpassung der Dämpferkompression.

# Felder

- `slot`: Position im Layout-Grid.
- `lyt`: Inneres GridLayout zur Anordnung der UI-Komponenten.
- `title`: Titel oder Beschriftung des Abschnitts.
- `sg_compr`: SliderGrid zur Steuerung der Kompressionsdämpfung.

# Konstruktor

- `DamperSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct DamperSection <: AbstractDamperSection 
    #######| Grid Position
    slot::Any

    #######| Components
    lyt::Union{GridLayout,Nothing}

    title::Union{Label,Nothing}

    sg_compr::Union{SliderGrid,Nothing}

    function DamperSection()
        inst = new()
        
        inst.slot = nothing

        inst.lyt = nothing
        inst.title = nothing
        inst.sg_compr = nothing

        return inst
    end
end

"""
    PlotSettingsSection <: AbstractPlotSettingsSection

Ein GUI-Abschnitt für Einstellungen zur Plotspeicherung und -verwaltung.  
Diese Sektion bietet Buttons zum Zurücksetzen, Speichern und Speichern aller Konfigurationen, sowie ein optionales Menü für weitere Optionen.

# Felder

- `slot`: Position im äußeren Layout-Grid.
- `lyt`: Hauptlayout zur Platzierung der Komponenten.
- `title`: Titel oder Überschrift des Abschnitts.
- `suplyt`: Sekundäres GridLayout zur Strukturierung von Buttons und Menü.
- `menu`: Menü für zusätzliche Einstellungen oder Optionen.
- `btn_reset`: Button zum Zurücksetzen der aktuellen Einstellungen.
- `btn_save`: Button zum Speichern aktueller Plot-Einstellungen.
- `btn_save_all`: Button zum Speichern aller aktuellen Einstellungen.

# Konstruktor

- `PlotSettingsSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct PlotSettingsSection <: AbstractPlotSettingsSection 
    #######| Grid Position
    slot::Any

    #######| Components
    lyt::Union{GridLayout,Nothing}
    title::Union{Label,Nothing}

    suplyt::Union{GridLayout,Nothing}
    menu::Union{Menu, Nothing}
    btn_reset::Union{Button, Nothing}
    btn_save::Union{Button, Nothing}
    btn_save_all::Union{Button, Nothing}

    function PlotSettingsSection()
        inst = new()
        
        inst.slot = nothing

        inst.lyt = nothing
        inst.title = nothing

        inst.suplyt = nothing
        inst.btn_reset = nothing
        inst.btn_save = nothing
        inst.btn_save_all = nothing

        return inst
    end
end

"""
    InfoSection <: AbstractInfoSection

Ein GUI-Abschnitt zur Anzeige von Informationen und Messwerten in Textboxen.  
Diese Sektion ist für die kompakte Darstellung von berechneten oder beobachteten Werten gedacht – z. B. Objektbezeichnungen, Radien oder Verhältnisangaben.

# Felder

- `slot`: Position im äußeren Layout-Grid.
- `lyt`: Hauptlayout zur Platzierung der Komponenten.
- `title`: Titel oder Überschrift des Abschnitts.
- `suplyt`: Sekundäres GridLayout zur Anordnung der Textboxen.

## Textboxen

- `tb_obj`: Anzeige des aktuellen Objektnamens oder Typs.
- `tb_δi`: Anzeige des inneren Δ-Wertes.
- `tb_δo`: Anzeige des äußeren Δ-Wertes.
- `tb_rad`: Anzeige des berechneten Radius.
- `tb_ratio`: Anzeige des Verhältnisses.

# Konstruktor

- `InfoSection()`: Erstellt eine neue Instanz mit allen Feldern auf `nothing` gesetzt.
"""
mutable struct InfoSection <: AbstractInfoSection 
    #######| Grid Position
    slot::Any

    #######| Components
    lyt::Union{GridLayout,Nothing}
    title::Union{Label,Nothing}

    suplyt::Union{GridLayout,Nothing}

    # Textboxes
    tb_obj::Union{Textbox, Nothing}
    tb_δi::Union{Textbox, Nothing}
    tb_δo::Union{Textbox, Nothing}
    tb_rad::Union{Textbox, Nothing}
    tb_ratio::Union{Textbox, Nothing}

    function InfoSection()
        inst = new()
        
        inst.slot = nothing 

        inst.lyt = nothing
        inst.title = nothing

        inst.suplyt = nothing
        inst.tb_obj = nothing
        inst.tb_δi = nothing
        inst.tb_δo = nothing
        inst.tb_rad = nothing
        inst.tb_ratio = nothing

        return inst
    end
end


"""
    InteractionLyt <: AbstractInteractionLyt

Ein übergeordneter Layout-Container, der alle interaktiven Sektionen der GUI zusammenfasst.  
Diese Struktur dient als zentrales Element zur Verwaltung und Darstellung aller Teilbereiche wie Plot, Parametersteuerung und Informationsanzeige.

# Felder

- `fig`: Die Hauptfigur (`Figure`), in der alle Sektionen eingebettet sind.
- `section_plot`: Sektion zur Anzeige und Beobachtung von Plots.
- `section_angle`: Sektion zur Einstellung von Winkeln.
- `section_param`: Sektion zur Steuerung von Modellparametern.
- `section_damper`: Sektion für Dämpfungseinstellungen (Kompression).
- `section_plot_settings`: Sektion zum Speichern, Zurücksetzen und Verwalten von Ploteinstellungen.
- `section_info`: Sektion zur Anzeige berechneter Werte und Objektdaten.

# Konstruktor

- `InteractionLyt()`: Erstellt eine neue Instanz mit allen Komponenten auf `nothing` gesetzt.
"""
mutable struct InteractionLyt <: AbstractInteractionLyt

    fig::Union{Figure,Nothing}

    section_plot::Union{PlotSection,Nothing}
    section_angle::Union{AngleSection,Nothing}
    section_param::Union{ParamSection,Nothing}
    section_damper::Union{DamperSection,Nothing}
    section_plot_settings::Union{PlotSettingsSection,Nothing}
    section_info::Union{InfoSection,Nothing}

    function InteractionLyt()
        inst = new()

        inst.fig = nothing

        inst.section_plot = nothing 
        inst.section_angle = nothing 
        inst.section_param = nothing 
        inst.section_damper = nothing 
        inst.section_plot_settings = nothing 
        inst.section_info = nothing 
        
        return inst
    end

end



