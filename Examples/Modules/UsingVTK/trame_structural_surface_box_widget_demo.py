#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
# SPDX-License-Identifier: BSD-3-Clause

"""
Trame test application for vtkStructuralSurfaceBoxWidget.

This script is meant for validating the widget against an installed VTK Python
package. It uses a VtkRemoteLocalView so the same scene can be inspected in
remote rendering mode and in local geometry mode.
"""

from __future__ import annotations

import math

import vtkmodules.vtkInteractionStyle  # noqa: F401
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401
from trame.app import get_server
from trame.ui.vuetify3 import SinglePageLayout
from trame.widgets import html, vtk as vtk_widgets, vuetify3 as v3
from vtkmodules.vtkCommonCore import vtkCommand, vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData, vtkPolyDataCollection, vtkPolyLine
from vtkmodules.vtkFiltersSources import vtkPlaneSource
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkInteractionWidgets import (
    vtkStructuralSurfaceBoxRepresentation,
    vtkStructuralSurfaceBoxWidget,
)
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer,
)


def make_structural_surface(z_base, ax, ay, wave, resolution):
    plane = vtkPlaneSource()
    plane.SetOrigin(-60.0, -60.0, 0.0)
    plane.SetPoint1(60.0, -60.0, 0.0)
    plane.SetPoint2(-60.0, 60.0, 0.0)
    plane.SetXResolution(resolution)
    plane.SetYResolution(resolution)
    plane.Update()

    surface = vtkPolyData()
    surface.DeepCopy(plane.GetOutput())
    points = surface.GetPoints()
    for idx in range(points.GetNumberOfPoints()):
        x, y, _ = points.GetPoint(idx)
        z = (
            z_base
            + ax * x
            + ay * y
            + wave * math.sin(0.08 * x) * math.cos(0.11 * y)
            + 0.45 * wave * math.cos(0.03 * (x + y))
        )
        points.SetPoint(idx, x, y, z)

    points.Modified()
    surface.Modified()
    return surface


def make_outline_trajectory():
    points = vtkPoints()
    points.InsertNextPoint(-50.0, -10.0, 28.0)
    points.InsertNextPoint(-20.0, 10.0, 30.0)
    points.InsertNextPoint(0.0, 0.0, 33.0)
    points.InsertNextPoint(25.0, 18.0, 37.0)
    points.InsertNextPoint(50.0, 5.0, 39.0)

    line = vtkPolyLine()
    line.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
    for idx in range(points.GetNumberOfPoints()):
        line.GetPointIds().SetId(idx, idx)

    cells = vtkCellArray()
    cells.InsertNextCell(line)

    poly_data = vtkPolyData()
    poly_data.SetPoints(points)
    poly_data.SetLines(cells)
    return poly_data


def make_surface_actor(surface, color, opacity):
    mapper = vtkPolyDataMapper()
    mapper.SetInputData(surface)

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetRepresentationToWireframe()
    actor.GetProperty().SetLineWidth(1.0)
    return actor


def make_trajectory_actor(trajectory):
    mapper = vtkPolyDataMapper()
    mapper.SetInputData(trajectory)

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1.0, 1.0, 1.0)
    actor.GetProperty().SetLineWidth(3.0)
    return actor


class StructuralSurfaceBoxTrameApp:
    def __init__(self):
        self.server = get_server(client_type="vue3")
        self.state = self.server.state
        self.ctrl = self.server.controller

        self._build_vtk_pipeline()
        self._bind_widget_events()
        self._build_ui()
        self._update_shell_state("ready")

    def _build_vtk_pipeline(self):
        renderer = vtkRenderer()
        renderer.SetBackground(0.93, 0.95, 0.98)

        render_window = vtkRenderWindow()
        render_window.SetWindowName("Trame Structural Surface Box Widget Demo")
        render_window.SetSize(1600, 900)
        render_window.AddRenderer(renderer)
        render_window.OffScreenRenderingOn()

        interactor = vtkRenderWindowInteractor()
        interactor.SetRenderWindow(render_window)
        interactor.SetInteractorStyle(vtkInteractorStyleTrackballCamera())

        surfaces = vtkPolyDataCollection()
        s0 = make_structural_surface(12.0, 0.10, -0.06, 5.5, 90)
        s1 = make_structural_surface(42.0, 0.05, 0.03, 7.5, 90)
        s2 = make_structural_surface(82.0, -0.03, 0.07, 6.5, 90)
        surfaces.AddItem(s0)
        surfaces.AddItem(s1)
        surfaces.AddItem(s2)

        palette = ((0.25, 0.45, 0.95), (0.20, 0.75, 0.35), (0.95, 0.45, 0.25))
        renderer.AddActor(make_surface_actor(s0, palette[0], 0.28))
        renderer.AddActor(make_surface_actor(s1, palette[1], 0.28))
        renderer.AddActor(make_surface_actor(s2, palette[2], 0.28))
        renderer.AddActor(make_trajectory_actor(make_outline_trajectory()))

        rep = vtkStructuralSurfaceBoxRepresentation()
        rep.SetRenderer(renderer)
        rep.SetStructuralSurfaces(surfaces)
        rep.SetActiveSurfaceIndex(0)
        rep.SetLowerSurfaceIndex(2)
        rep.SetSamplingDimensions(25, 25, 9)
        rep.SetTopInterpolation(0.18)
        rep.SetBottomInterpolation(0.82)
        rep.SetHandleRadius(1.0)
        rep.SetFootprint(-20.0, 20.0, -24.0, 24.0)
        rep.BuildRepresentation()

        widget = vtkStructuralSurfaceBoxWidget()
        widget.SetInteractor(interactor)
        widget.SetCurrentRenderer(renderer)
        widget.SetPriority(1.0)
        widget.SetRepresentation(rep)
        widget.SetEnabled(1)

        renderer.ResetCamera()
        renderer.GetActiveCamera().Elevation(25.0)
        renderer.GetActiveCamera().Azimuth(30.0)
        renderer.ResetCameraClippingRange()
        render_window.Render()

        self.renderer = renderer
        self.render_window = render_window
        self.interactor = interactor
        self.rep = rep
        self.widget = widget

    def _update_shell_state(self, event_name):
        shell = self.rep.GetClosedSurface()
        dims = [0, 0, 0]
        self.rep.GetSamplingDimensions(dims)
        self.state.shell_points = shell.GetNumberOfPoints()
        self.state.shell_cells = shell.GetNumberOfCells()
        self.state.sampling_dims = f"{dims[0]} x {dims[1]} x {dims[2]}"
        self.state.top_interpolation = round(self.rep.GetTopInterpolation(), 4)
        self.state.bottom_interpolation = round(self.rep.GetBottomInterpolation(), 4)
        self.state.last_widget_event = event_name
        if callable(getattr(self.ctrl, "view_update", None)):
            self.ctrl.view_update()

    def _bind_widget_events(self):
        def observer(label):
            def _callback(*_):
                self._update_shell_state(label)

            return _callback

        self.widget.AddObserver(vtkCommand.StartInteractionEvent, observer("start"))
        self.widget.AddObserver(vtkCommand.InteractionEvent, observer("interaction"))
        self.widget.AddObserver(vtkCommand.EndInteractionEvent, observer("end"))
        self.widget.AddObserver(self.widget.TranslateStartEvent, observer("translate-start"))
        self.widget.AddObserver(self.widget.TranslateInteractionEvent, observer("translate"))
        self.widget.AddObserver(self.widget.TranslateEndEvent, observer("translate-end"))
        self.widget.AddObserver(self.widget.ResizeStartEvent, observer("resize-start"))
        self.widget.AddObserver(self.widget.ResizeInteractionEvent, observer("resize"))
        self.widget.AddObserver(self.widget.ResizeEndEvent, observer("resize-end"))

    def _build_ui(self):
        with SinglePageLayout(self.server) as layout:
            layout.title.set_text("Structural Surface Box Widget / Trame")

            with layout.toolbar:
                html.Div(
                    "Use remote mode to validate server-side widget interaction. Local mode is useful to inspect "
                    "the exported scene in the browser.",
                    classes="text-caption mx-4",
                )
                v3.VSpacer()
                v3.VSelect(
                    v_model=("viewMode", "remote"),
                    items=("view_modes", [{"title": "Remote", "value": "remote"}, {"title": "Local", "value": "local"}]),
                    density="compact",
                    hide_details=True,
                    style="max-width: 170px",
                )
                v3.VBtn("Reset camera", click=self.ctrl.view_reset_camera, classes="ml-2", density="compact")

            with layout.content:
                with v3.VContainer(fluid=True, classes="fill-height pa-0 ma-0"):
                    with v3.VRow(classes="fill-height ma-0", dense=True):
                        with v3.VCol(cols=9, classes="pa-0 fill-height"):
                            view = vtk_widgets.VtkRemoteLocalView(
                                self.render_window,
                                namespace="view",
                                mode="remote",
                                interactive_ratio=1,
                            )
                            self.ctrl.view_update = view.update
                            self.ctrl.view_reset_camera = view.reset_camera

                        with v3.VCol(cols=3, classes="pa-2"):
                            with v3.VCard(variant="outlined", classes="mb-2"):
                                v3.VCardTitle("Widget state")
                                with v3.VCardText():
                                    html.Div("Last widget event: {{ last_widget_event }}")
                                    html.Div("Shell points: {{ shell_points }}")
                                    html.Div("Shell cells: {{ shell_cells }}")
                                    html.Div("Sampling dims: {{ sampling_dims }}")
                                    html.Div("Top interpolation: {{ top_interpolation }}")
                                    html.Div("Bottom interpolation: {{ bottom_interpolation }}")

                            with v3.VCard(variant="outlined"):
                                v3.VCardTitle("Windows setup hint")
                                with v3.VCardText(classes="text-body-2"):
                                    html.Pre(
                                        "set PYTHONPATH=D:\\dev\\vtk\\install\\Lib\\site-packages;%PYTHONPATH%\n"
                                        "python trame_structural_surface_box_widget_demo.py",
                                        style="white-space: pre-wrap;",
                                    )

    def start(self, **kwargs):
        self.server.start(**kwargs)


def main():
    StructuralSurfaceBoxTrameApp().start()


if __name__ == "__main__":
    main()
