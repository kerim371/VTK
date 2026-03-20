// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

#include "vtkBoreholeRepresentation.h"
#include "vtkBoreholeWidget.h"

#include "vtkCellArray.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkNew.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyLine.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

namespace
{
vtkSmartPointer<vtkPolyData> MakeTrajectory()
{
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(0.2, 0.0, 10.0);
  points->InsertNextPoint(1.0, 0.5, 20.0);
  points->InsertNextPoint(2.0, 1.0, 30.0);
  points->InsertNextPoint(2.2, 1.5, 40.0);

  vtkNew<vtkPolyLine> line;
  line->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i)
  {
    line->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> lines;
  lines->InsertNextCell(line);

  vtkNew<vtkPolyData> trajectory;
  trajectory->SetPoints(points);
  trajectory->SetLines(lines);

  return trajectory;
}
}

int main(int, char*[])
{
  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(0.12, 0.14, 0.18);

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetWindowName("VTK Borehole Widget Demo");
  renderWindow->SetSize(1200, 800);
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  vtkNew<vtkBoreholeRepresentation> rep;
  rep->SetRenderer(renderer);
  rep->SetInputData(MakeTrajectory());
  rep->SetRadius(0.6);
  rep->SetTopPosition(0.15);
  rep->SetBottomPosition(0.85);
  rep->BuildRepresentation();

  vtkNew<vtkBoreholeWidget> widget;
  widget->SetInteractor(interactor);
  widget->SetCurrentRenderer(renderer);
  widget->SetPriority(1.0);
  widget->SetRepresentation(rep);
  widget->SetEnabled(1);

  renderer->ResetCamera();
  renderWindow->Render();
  interactor->Start();

  return 0;
}
