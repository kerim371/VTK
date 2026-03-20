// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkBoreholeRepresentation.h"
#include "vtkBoreholeWidget.h"

#include "vtkCellArray.h"
#include "vtkMath.h"
#include "vtkNew.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyLine.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

#include <cstdlib>
#include <iostream>

int TestBoreholeWidget(int, char*[])
{
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 10.0);
  points->InsertNextPoint(1.0, 0.5, 20.0);
  points->InsertNextPoint(2.0, 1.0, 30.0);

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

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->OffScreenRenderingOn();
  renderWindow->SetSize(300, 300);
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkBoreholeRepresentation> rep;
  rep->SetRenderer(renderer);
  rep->SetInputData(trajectory);
  rep->SetRadius(0.5);
  rep->SetTopPosition(0.2);
  rep->SetBottomPosition(0.8);
  rep->BuildRepresentation();

  double* bounds = rep->GetBounds();
  if (!bounds || !vtkMath::AreBoundsInitialized(bounds))
  {
    std::cerr << "Borehole representation produced invalid bounds." << std::endl;
    return EXIT_FAILURE;
  }

  vtkNew<vtkBoreholeWidget> widget;
  widget->SetInteractor(interactor);
  widget->SetCurrentRenderer(renderer);
  widget->SetRepresentation(rep);
  widget->SetEnabled(1);

  renderWindow->Render();

  // Exercise interaction API (no event playback required).
  rep->ComputeInteractionState(150, 150);
  rep->SetCurrentOperation(vtkBoreholeRepresentation::DragWallRadius);
  double e[2] = { 150.0, 150.0 };
  rep->StartWidgetInteraction(e);
  rep->WidgetInteraction(e);
  rep->EndWidgetInteraction(e);

  widget->SetEnabled(0);

  return EXIT_SUCCESS;
}
