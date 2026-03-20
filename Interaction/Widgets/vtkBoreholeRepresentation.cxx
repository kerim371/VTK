// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkBoreholeRepresentation.h"

#include "vtkActor.h"
#include "vtkCellPicker.h"
#include "vtkClipClosedSurface.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPlane.h"
#include "vtkPlaneCollection.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPoints.h"
#include "vtkProperty.h"
#include "vtkPropCollection.h"
#include "vtkRegularPolygonSource.h"
#include "vtkRenderer.h"
#include "vtkTubeFilter.h"
#include "vtkViewport.h"
#include "vtkWindow.h"

#include <algorithm>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkBoreholeRepresentation);

vtkBoreholeRepresentation::vtkBoreholeRepresentation()
  : Input(nullptr)
  , Radius(1.0)
  , TopPosition(0.1)
  , BottomPosition(0.9)
  , TotalLength(0.0)
  , CurrentOperation(DragNone)
{
  this->Tube = vtkTubeFilter::New();
  this->Tube->CappingOn();
  this->Tube->SetNumberOfSides(32);
  this->Tube->SetRadius(this->Radius);

  this->Planes = vtkPlaneCollection::New();
  this->TopPlane = vtkPlane::New();
  this->BottomPlane = vtkPlane::New();
  this->Planes->AddItem(this->TopPlane);
  this->Planes->AddItem(this->BottomPlane);

  this->Clip = vtkClipClosedSurface::New();
  this->Clip->SetInputConnection(this->Tube->GetOutputPort());
  this->Clip->SetClippingPlanes(this->Planes);

  this->WallMapper = vtkPolyDataMapper::New();
  this->WallMapper->SetInputConnection(this->Clip->GetOutputPort());

  this->WallActor = vtkActor::New();
  this->WallActor->SetMapper(this->WallMapper);

  this->TopCapSource = vtkRegularPolygonSource::New();
  this->TopCapSource->SetNumberOfSides(48);
  this->TopCapSource->GeneratePolygonOn();
  this->TopCapSource->SetRadius(this->Radius);
  this->TopCapMapper = vtkPolyDataMapper::New();
  this->TopCapMapper->SetInputConnection(this->TopCapSource->GetOutputPort());
  this->TopCapActor = vtkActor::New();
  this->TopCapActor->SetMapper(this->TopCapMapper);

  this->BottomCapSource = vtkRegularPolygonSource::New();
  this->BottomCapSource->SetNumberOfSides(48);
  this->BottomCapSource->GeneratePolygonOn();
  this->BottomCapSource->SetRadius(this->Radius);
  this->BottomCapMapper = vtkPolyDataMapper::New();
  this->BottomCapMapper->SetInputConnection(this->BottomCapSource->GetOutputPort());
  this->BottomCapActor = vtkActor::New();
  this->BottomCapActor->SetMapper(this->BottomCapMapper);

  this->DefaultWallProperty = vtkProperty::New();
  this->DefaultWallProperty->SetColor(0.8, 0.8, 0.9);
  this->SelectedWallProperty = vtkProperty::New();
  this->SelectedWallProperty->SetColor(1.0, 0.8, 0.2);

  this->DefaultCapProperty = vtkProperty::New();
  this->DefaultCapProperty->SetColor(0.7, 0.8, 1.0);
  this->DefaultCapProperty->SetOpacity(0.6);
  this->SelectedCapProperty = vtkProperty::New();
  this->SelectedCapProperty->SetColor(1.0, 0.4, 0.1);
  this->SelectedCapProperty->SetOpacity(0.8);

  this->WallActor->SetProperty(this->DefaultWallProperty);
  this->TopCapActor->SetProperty(this->DefaultCapProperty);
  this->BottomCapActor->SetProperty(this->DefaultCapProperty);

  this->Picker = vtkCellPicker::New();
  this->Picker->SetTolerance(0.005);

  this->LastEventPosition[0] = 0.0;
  this->LastEventPosition[1] = 0.0;
}

vtkBoreholeRepresentation::~vtkBoreholeRepresentation()
{
  this->SetInputData(nullptr);
  this->Tube->Delete();
  this->Clip->Delete();
  this->Planes->Delete();
  this->TopPlane->Delete();
  this->BottomPlane->Delete();
  this->WallMapper->Delete();
  this->WallActor->Delete();
  this->TopCapSource->Delete();
  this->TopCapMapper->Delete();
  this->TopCapActor->Delete();
  this->BottomCapSource->Delete();
  this->BottomCapMapper->Delete();
  this->BottomCapActor->Delete();
  this->DefaultWallProperty->Delete();
  this->SelectedWallProperty->Delete();
  this->DefaultCapProperty->Delete();
  this->SelectedCapProperty->Delete();
  this->Picker->Delete();
}

void vtkBoreholeRepresentation::SetInputData(vtkPolyData* polyData)
{
  if (this->Input == polyData)
  {
    return;
  }

  if (this->Input)
  {
    this->Input->UnRegister(this);
  }

  this->Input = polyData;
  if (this->Input)
  {
    this->Input->Register(this);
    this->Tube->SetInputData(this->Input);
  }
  else
  {
    this->Tube->SetInputData(nullptr);
  }

  this->Modified();
}

void vtkBoreholeRepresentation::SetCurrentOperation(int op)
{
  if (this->CurrentOperation == op)
  {
    return;
  }
  this->CurrentOperation = op;
  this->Modified();
}

void vtkBoreholeRepresentation::BuildRepresentation()
{
  if (!this->Input || !this->Input->GetPoints() || this->Input->GetNumberOfPoints() < 2)
  {
    return;
  }

  this->Tube->SetRadius(this->Radius);

  vtkPoints* points = this->Input->GetPoints();
  const vtkIdType npts = points->GetNumberOfPoints();

  this->CumulativeLengths.resize(static_cast<size_t>(npts));
  this->CumulativeLengths[0] = 0.0;
  this->TotalLength = 0.0;
  for (vtkIdType i = 1; i < npts; ++i)
  {
    double p0[3];
    double p1[3];
    points->GetPoint(i - 1, p0);
    points->GetPoint(i, p1);
    this->TotalLength += std::sqrt(vtkMath::Distance2BetweenPoints(p0, p1));
    this->CumulativeLengths[static_cast<size_t>(i)] = this->TotalLength;
  }

  if (this->TotalLength <= 0.0)
  {
    return;
  }

  const double minGap = 1e-4;
  this->TopPosition = std::clamp(this->TopPosition, 0.0, 1.0 - minGap);
  this->BottomPosition = std::clamp(this->BottomPosition, minGap, 1.0);
  if (this->TopPosition >= this->BottomPosition - minGap)
  {
    this->BottomPosition = std::min(1.0, this->TopPosition + minGap);
  }

  this->UpdateClippingPlanes();
  this->UpdateCapActors();

  this->Tube->Update();
  this->Clip->Update();

  // If clipping produced an empty output, try flipping both clipping plane
  // normals. This makes the representation resilient to plane-orientation
  // convention differences.
  if (this->Clip->GetOutput() && this->Clip->GetOutput()->GetNumberOfPoints() == 0)
  {
    double nTop[3];
    double nBottom[3];
    this->TopPlane->GetNormal(nTop);
    this->BottomPlane->GetNormal(nBottom);
    this->TopPlane->SetNormal(-nTop[0], -nTop[1], -nTop[2]);
    this->BottomPlane->SetNormal(-nBottom[0], -nBottom[1], -nBottom[2]);
    this->Clip->Update();
  }
}

bool vtkBoreholeRepresentation::ComputePointAndTangent(double t, double point[3], double tangent[3]) const
{
  if (!this->Input || !this->Input->GetPoints() || this->Input->GetNumberOfPoints() < 2 ||
    this->TotalLength <= 0.0)
  {
    return false;
  }

  vtkPoints* pts = this->Input->GetPoints();
  const vtkIdType npts = pts->GetNumberOfPoints();

  const double target = std::clamp(t, 0.0, 1.0) * this->TotalLength;
  vtkIdType segId = 0;
  for (vtkIdType i = 1; i < npts; ++i)
  {
    if (this->CumulativeLengths[static_cast<size_t>(i)] >= target)
    {
      segId = i - 1;
      break;
    }
    segId = i - 1;
  }

  double p0[3];
  double p1[3];
  pts->GetPoint(segId, p0);
  pts->GetPoint(segId + 1, p1);

  const double segStart = this->CumulativeLengths[static_cast<size_t>(segId)];
  const double segEnd = this->CumulativeLengths[static_cast<size_t>(segId + 1)];
  const double segLen = std::max(segEnd - segStart, 1e-12);
  const double u = std::clamp((target - segStart) / segLen, 0.0, 1.0);

  for (int i = 0; i < 3; ++i)
  {
    point[i] = p0[i] + u * (p1[i] - p0[i]);
    tangent[i] = p1[i] - p0[i];
  }
  vtkMath::Normalize(tangent);
  return true;
}

void vtkBoreholeRepresentation::UpdateClippingPlanes()
{
  double topPoint[3];
  double topTangent[3];
  double bottomPoint[3];
  double bottomTangent[3];
  if (!this->ComputePointAndTangent(this->TopPosition, topPoint, topTangent) ||
    !this->ComputePointAndTangent(this->BottomPosition, bottomPoint, bottomTangent))
  {
    return;
  }

  // Keep the interval between planes (top keeps segment towards +t, bottom towards -t).
  this->TopPlane->SetOrigin(topPoint);
  this->TopPlane->SetNormal(-topTangent[0], -topTangent[1], -topTangent[2]);

  this->BottomPlane->SetOrigin(bottomPoint);
  this->BottomPlane->SetNormal(bottomTangent);
}

void vtkBoreholeRepresentation::UpdateCapActors()
{
  double topPoint[3];
  double topTangent[3];
  double bottomPoint[3];
  double bottomTangent[3];
  if (!this->ComputePointAndTangent(this->TopPosition, topPoint, topTangent) ||
    !this->ComputePointAndTangent(this->BottomPosition, bottomPoint, bottomTangent))
  {
    return;
  }

  this->TopCapSource->SetCenter(topPoint);
  this->TopCapSource->SetNormal(topTangent);
  this->TopCapSource->SetRadius(this->Radius);

  this->BottomCapSource->SetCenter(bottomPoint);
  this->BottomCapSource->SetNormal(-bottomTangent[0], -bottomTangent[1], -bottomTangent[2]);
  this->BottomCapSource->SetRadius(this->Radius);
}

bool vtkBoreholeRepresentation::PickWorldPoint(int X, int Y, double worldPt[3])
{
  if (!this->Renderer)
  {
    return false;
  }

  if (!this->Picker->Pick(static_cast<double>(X), static_cast<double>(Y), 0.0, this->Renderer))
  {
    return false;
  }

  this->Picker->GetPickPosition(worldPt);
  return true;
}

bool vtkBoreholeRepresentation::ComputeClosestOnTrajectory(
  const double worldPt[3], double& t, double& distance) const
{
  if (!this->Input || !this->Input->GetPoints() || this->Input->GetNumberOfPoints() < 2 ||
    this->TotalLength <= 0.0)
  {
    return false;
  }

  vtkPoints* pts = this->Input->GetPoints();
  const vtkIdType npts = pts->GetNumberOfPoints();

  distance = VTK_DOUBLE_MAX;
  t = this->TopPosition;

  for (vtkIdType i = 0; i < npts - 1; ++i)
  {
    double p0[3];
    double p1[3];
    pts->GetPoint(i, p0);
    pts->GetPoint(i + 1, p1);

    double v[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
    double w[3] = { worldPt[0] - p0[0], worldPt[1] - p0[1], worldPt[2] - p0[2] };
    const double vv = vtkMath::Dot(v, v);
    if (vv <= 0.0)
    {
      continue;
    }
    const double u = std::clamp(vtkMath::Dot(w, v) / vv, 0.0, 1.0);
    double c[3] = { p0[0] + u * v[0], p0[1] + u * v[1], p0[2] + u * v[2] };
    const double d = std::sqrt(vtkMath::Distance2BetweenPoints(worldPt, c));

    if (d < distance)
    {
      distance = d;
      const double s0 = this->CumulativeLengths[static_cast<size_t>(i)];
      const double s1 = this->CumulativeLengths[static_cast<size_t>(i + 1)];
      const double s = s0 + u * (s1 - s0);
      t = s / this->TotalLength;
    }
  }

  return distance < VTK_DOUBLE_MAX;
}

int vtkBoreholeRepresentation::ComputeInteractionState(int X, int Y, int vtkNotUsed(modify))
{
  if (!this->Renderer)
  {
    this->InteractionState = Outside;
    return this->InteractionState;
  }

  this->BuildRepresentation();

  int state = Outside;
  if (this->Picker->Pick(static_cast<double>(X), static_cast<double>(Y), 0.0, this->Renderer))
  {
    vtkProp* prop = this->Picker->GetViewProp();
    if (prop == this->TopCapActor)
    {
      state = OverTopCap;
    }
    else if (prop == this->BottomCapActor)
    {
      state = OverBottomCap;
    }
    else if (prop == this->WallActor)
    {
      state = OverWall;
    }
  }

  this->InteractionState = state;
  this->HighlightPart(state);
  return state;
}

void vtkBoreholeRepresentation::HighlightPart(int state)
{
  this->WallActor->SetProperty(state == OverWall ? this->SelectedWallProperty : this->DefaultWallProperty);
  this->TopCapActor->SetProperty(state == OverTopCap ? this->SelectedCapProperty : this->DefaultCapProperty);
  this->BottomCapActor->SetProperty(
    state == OverBottomCap ? this->SelectedCapProperty : this->DefaultCapProperty);
}

void vtkBoreholeRepresentation::StartWidgetInteraction(double eventPos[2])
{
  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];
}

void vtkBoreholeRepresentation::WidgetInteraction(double newEventPos[2])
{
  this->LastEventPosition[0] = newEventPos[0];
  this->LastEventPosition[1] = newEventPos[1];

  if (this->CurrentOperation == DragNone)
  {
    return;
  }

  double worldPt[3];
  if (!this->PickWorldPoint(static_cast<int>(newEventPos[0]), static_cast<int>(newEventPos[1]), worldPt))
  {
    return;
  }

  double t = 0.0;
  double distance = 0.0;
  if (!this->ComputeClosestOnTrajectory(worldPt, t, distance))
  {
    return;
  }

  const double minGap = 1e-4;
  if (this->CurrentOperation == DragTopCap)
  {
    this->TopPosition = std::clamp(t, 0.0, this->BottomPosition - minGap);
  }
  else if (this->CurrentOperation == DragBottomCap)
  {
    this->BottomPosition = std::clamp(t, this->TopPosition + minGap, 1.0);
  }
  else if (this->CurrentOperation == DragWallRadius)
  {
    this->Radius = std::max(distance, 1e-6);
  }

  this->BuildRepresentation();
  this->NeedToRenderOn();
}

void vtkBoreholeRepresentation::EndWidgetInteraction(double vtkNotUsed(newEventPos)[2])
{
  this->CurrentOperation = DragNone;
}

double* vtkBoreholeRepresentation::GetBounds()
{
  this->BuildRepresentation();
  return this->WallActor->GetBounds();
}

void vtkBoreholeRepresentation::GetActors(vtkPropCollection* pc)
{
  pc->AddItem(this->WallActor);
  pc->AddItem(this->TopCapActor);
  pc->AddItem(this->BottomCapActor);
}

void vtkBoreholeRepresentation::ReleaseGraphicsResources(vtkWindow* w)
{
  this->WallActor->ReleaseGraphicsResources(w);
  this->TopCapActor->ReleaseGraphicsResources(w);
  this->BottomCapActor->ReleaseGraphicsResources(w);
}

int vtkBoreholeRepresentation::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->BuildRepresentation();
  int count = 0;
  count += this->WallActor->RenderOpaqueGeometry(viewport);
  count += this->TopCapActor->RenderOpaqueGeometry(viewport);
  count += this->BottomCapActor->RenderOpaqueGeometry(viewport);
  return count;
}

int vtkBoreholeRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* viewport)
{
  int count = 0;
  count += this->WallActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->TopCapActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->BottomCapActor->RenderTranslucentPolygonalGeometry(viewport);
  return count;
}

vtkTypeBool vtkBoreholeRepresentation::HasTranslucentPolygonalGeometry()
{
  return this->WallActor->HasTranslucentPolygonalGeometry() ||
    this->TopCapActor->HasTranslucentPolygonalGeometry() ||
    this->BottomCapActor->HasTranslucentPolygonalGeometry();
}

void vtkBoreholeRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Radius: " << this->Radius << "\n";
  os << indent << "TopPosition: " << this->TopPosition << "\n";
  os << indent << "BottomPosition: " << this->BottomPosition << "\n";
}
VTK_ABI_NAMESPACE_END
