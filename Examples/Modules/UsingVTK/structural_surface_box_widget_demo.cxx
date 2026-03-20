// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkNew.h"
#include "vtkPlaneSource.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataCollection.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyLine.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkStructuralSurfaceBoxRepresentation.h"
#include "vtkStructuralSurfaceBoxWidget.h"

#include <array>
#include <cmath>

namespace
{
vtkSmartPointer<vtkPolyData> MakeStructuralSurface(
  double zBase, double ax, double ay, double wave, int resolution)
{
  vtkNew<vtkPlaneSource> plane;
  plane->SetOrigin(-60.0, -60.0, 0.0);
  plane->SetPoint1(60.0, -60.0, 0.0);
  plane->SetPoint2(-60.0, 60.0, 0.0);
  plane->SetXResolution(resolution);
  plane->SetYResolution(resolution);
  plane->Update();

  vtkNew<vtkPolyData> surface;
  surface->DeepCopy(plane->GetOutput());
  vtkPoints* pts = surface->GetPoints();
  for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i)
  {
    double p[3];
    pts->GetPoint(i, p);
    p[2] = zBase + ax * p[0] + ay * p[1] +
      wave * std::sin(0.08 * p[0]) * std::cos(0.11 * p[1]) +
      0.45 * wave * std::cos(0.03 * (p[0] + p[1]));
    pts->SetPoint(i, p);
  }
  pts->Modified();
  surface->Modified();
  return surface;
}

vtkSmartPointer<vtkPolyData> MakeOutlineTrajectory()
{
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(-50.0, -10.0, 28.0);
  points->InsertNextPoint(-20.0, 10.0, 30.0);
  points->InsertNextPoint(0.0, 0.0, 33.0);
  points->InsertNextPoint(25.0, 18.0, 37.0);
  points->InsertNextPoint(50.0, 5.0, 39.0);

  vtkNew<vtkPolyLine> line;
  line->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    line->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> cells;
  cells->InsertNextCell(line);

  vtkNew<vtkPolyData> pd;
  pd->SetPoints(points);
  pd->SetLines(cells);
  return pd;
}

vtkSmartPointer<vtkActor> MakeSurfaceActor(vtkPolyData* surface, const double color[3], double opacity)
{
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(surface);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(color[0], color[1], color[2]);
  actor->GetProperty()->SetOpacity(opacity);
  actor->GetProperty()->SetRepresentationToWireframe();
  actor->GetProperty()->SetLineWidth(1.0);
  return actor;
}

vtkSmartPointer<vtkActor> MakeTrajectoryActor(vtkPolyData* trajectory)
{
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(trajectory);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
  actor->GetProperty()->SetLineWidth(3.0);
  return actor;
}
}

int main(int, char*[])
{
  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.93, 0.95, 0.98);

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetWindowName("VTK Structural Surface Box Widget Demo");
  renderWindow->SetSize(1400, 900);
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  vtkNew<vtkPolyDataCollection> surfaces;
  auto s0 = MakeStructuralSurface(12.0, 0.10, -0.06, 5.5, 90);
  auto s1 = MakeStructuralSurface(42.0, 0.05, 0.03, 7.5, 90);
  auto s2 = MakeStructuralSurface(82.0, -0.03, 0.07, 6.5, 90);
  surfaces->AddItem(s0);
  surfaces->AddItem(s1);
  surfaces->AddItem(s2);

  const std::array<std::array<double, 3>, 3> palette = {
    std::array<double, 3>{ 0.25, 0.45, 0.95 }, std::array<double, 3>{ 0.20, 0.75, 0.35 },
    std::array<double, 3>{ 0.95, 0.45, 0.25 }
  };
  renderer->AddActor(MakeSurfaceActor(s0, palette[0].data(), 0.28));
  renderer->AddActor(MakeSurfaceActor(s1, palette[1].data(), 0.28));
  renderer->AddActor(MakeSurfaceActor(s2, palette[2].data(), 0.28));
  renderer->AddActor(MakeTrajectoryActor(MakeOutlineTrajectory()));

  vtkNew<vtkStructuralSurfaceBoxRepresentation> rep;
  rep->SetRenderer(renderer);
  rep->SetStructuralSurfaces(surfaces);
  rep->SetActiveSurfaceIndex(1);
  rep->SetSamplingResolutionX(24);
  rep->SetSamplingResolutionY(24);
  rep->SetFootprint(-20.0, 20.0, -24.0, 24.0);
  rep->SetBottomZ(12.0);
  rep->BuildRepresentation();

  vtkNew<vtkStructuralSurfaceBoxWidget> widget;
  widget->SetInteractor(interactor);
  widget->SetCurrentRenderer(renderer);
  widget->SetPriority(1.0);
  widget->SetRepresentation(rep);
  widget->SetEnabled(1);

  renderer->ResetCamera();
  renderer->GetActiveCamera()->Elevation(25.0);
  renderer->GetActiveCamera()->Azimuth(30.0);
  renderer->ResetCameraClippingRange();

  renderWindow->Render();
  interactor->Start();
  return 0;
}
