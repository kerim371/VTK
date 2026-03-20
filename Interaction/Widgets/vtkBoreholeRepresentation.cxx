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
#include "vtkPolyDataCollection.h"
#include "vtkPolyDataMapper.h"
#include "vtkPoints.h"
#include "vtkProperty.h"
#include "vtkPropCollection.h"
#include "vtkRegularPolygonSource.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkTubeFilter.h"
#include "vtkViewport.h"
#include "vtkWindow.h"

#include <algorithm>
#include <cmath>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkBoreholeRepresentation);

vtkBoreholeRepresentation::vtkBoreholeRepresentation()
  : Input(nullptr)
  , Radius(1.0)
  , TopPosition(0.1)
  , BottomPosition(0.9)
  , TotalLength(0.0)
  , ActiveT(0.0)
  , DragInitialized(false)
  , SurfaceDx(1.0)
  , SurfaceDy(1.0)
  , CurrentOperation(DragNone)
{
  this->Tube = vtkTubeFilter::New();
  this->Tube->CappingOn();
  this->Tube->SetNumberOfSides(32);
  this->Tube->SetRadius(this->Radius);

  this->Planes = vtkPlaneCollection::New();
  this->TopPlane = vtkPlane::New();
  this->BottomPlane = vtkPlane::New();
  this->StructuralSurfaces = nullptr;
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

  this->AxisMapper = vtkPolyDataMapper::New();
  this->AxisActor = vtkActor::New();
  this->AxisActor->SetMapper(this->AxisMapper);

  this->TopGlyphSource = vtkSphereSource::New();
  this->TopGlyphSource->SetThetaResolution(24);
  this->TopGlyphSource->SetPhiResolution(24);
  this->TopGlyphMapper = vtkPolyDataMapper::New();
  this->TopGlyphMapper->SetInputConnection(this->TopGlyphSource->GetOutputPort());
  this->TopGlyphActor = vtkActor::New();
  this->TopGlyphActor->SetMapper(this->TopGlyphMapper);

  this->BottomGlyphSource = vtkSphereSource::New();
  this->BottomGlyphSource->SetThetaResolution(24);
  this->BottomGlyphSource->SetPhiResolution(24);
  this->BottomGlyphMapper = vtkPolyDataMapper::New();
  this->BottomGlyphMapper->SetInputConnection(this->BottomGlyphSource->GetOutputPort());
  this->BottomGlyphActor = vtkActor::New();
  this->BottomGlyphActor->SetMapper(this->BottomGlyphMapper);

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

  this->AxisProperty = vtkProperty::New();
  this->AxisProperty->SetColor(0.95, 0.95, 0.95);
  this->AxisProperty->SetLineWidth(3.0);

  this->DefaultGlyphProperty = vtkProperty::New();
  this->DefaultGlyphProperty->SetColor(0.3, 1.0, 0.3);
  this->SelectedGlyphProperty = vtkProperty::New();
  this->SelectedGlyphProperty->SetColor(1.0, 1.0, 0.1);

  this->WallActor->SetProperty(this->DefaultWallProperty);
  this->TopCapActor->SetProperty(this->DefaultCapProperty);
  this->BottomCapActor->SetProperty(this->DefaultCapProperty);
  this->AxisActor->SetProperty(this->AxisProperty);
  this->TopGlyphActor->SetProperty(this->DefaultGlyphProperty);
  this->BottomGlyphActor->SetProperty(this->DefaultGlyphProperty);

  this->Picker = vtkCellPicker::New();
  this->Picker->SetTolerance(0.005);
  this->Picker->PickFromListOn();

  this->LastEventPosition[0] = 0.0;
  this->LastEventPosition[1] = 0.0;
  this->LastPickPosition[0] = 0.0;
  this->LastPickPosition[1] = 0.0;
  this->LastPickPosition[2] = 0.0;
}

vtkBoreholeRepresentation::~vtkBoreholeRepresentation()
{
  this->SetInputData(nullptr);
  this->SetStructuralSurfaces(nullptr);
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
  this->AxisMapper->Delete();
  this->AxisActor->Delete();
  this->TopGlyphSource->Delete();
  this->TopGlyphMapper->Delete();
  this->TopGlyphActor->Delete();
  this->BottomGlyphSource->Delete();
  this->BottomGlyphMapper->Delete();
  this->BottomGlyphActor->Delete();
  this->DefaultWallProperty->Delete();
  this->SelectedWallProperty->Delete();
  this->DefaultCapProperty->Delete();
  this->SelectedCapProperty->Delete();
  this->AxisProperty->Delete();
  this->DefaultGlyphProperty->Delete();
  this->SelectedGlyphProperty->Delete();
  this->Picker->Delete();
}

void vtkBoreholeRepresentation::SetStructuralSurfaces(vtkPolyDataCollection* surfaces)
{
  if (this->StructuralSurfaces == surfaces)
  {
    return;
  }

  if (this->StructuralSurfaces)
  {
    this->StructuralSurfaces->UnRegister(this);
  }
  this->StructuralSurfaces = surfaces;
  if (this->StructuralSurfaces)
  {
    this->StructuralSurfaces->Register(this);
  }
  this->Modified();
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
    this->AxisMapper->SetInputData(this->Input);
  }
  else
  {
    this->Tube->SetInputData(nullptr);
    this->AxisMapper->SetInputData(nullptr);
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
  this->UpdateGlyphActors();

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
    this->UpdateCapActors();
    this->UpdateGlyphActors();
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

  double topNormal[3] = { -topTangent[0], -topTangent[1], -topTangent[2] };
  double bottomNormal[3] = { bottomTangent[0], bottomTangent[1], bottomTangent[2] };
  this->ComputeInterpolatedSurfaceNormal(topPoint, topNormal);
  this->ComputeInterpolatedSurfaceNormal(bottomPoint, bottomNormal);

  // Keep the interval between planes (top keeps segment towards +t, bottom towards -t).
  this->TopPlane->SetOrigin(topPoint);
  this->TopPlane->SetNormal(topNormal);

  this->BottomPlane->SetOrigin(bottomPoint);
  this->BottomPlane->SetNormal(bottomNormal);
}

void vtkBoreholeRepresentation::UpdateCapActors()
{
  double topPoint[3];
  double bottomPoint[3];
  double topNormal[3];
  double bottomNormal[3];
  this->TopPlane->GetOrigin(topPoint);
  this->BottomPlane->GetOrigin(bottomPoint);
  this->TopPlane->GetNormal(topNormal);
  this->BottomPlane->GetNormal(bottomNormal);
  if (vtkMath::Norm(topNormal) <= 0.0 || vtkMath::Norm(bottomNormal) <= 0.0)
  {
    return;
  }

  this->TopCapSource->SetCenter(topPoint);
  this->TopCapSource->SetNormal(-topNormal[0], -topNormal[1], -topNormal[2]);
  this->TopCapSource->SetRadius(this->Radius);

  const double eps = std::max(1e-4, 1e-3 * this->Radius);
  topPoint[0] -= eps * topNormal[0];
  topPoint[1] -= eps * topNormal[1];
  topPoint[2] -= eps * topNormal[2];
  this->TopCapSource->SetCenter(topPoint);

  bottomPoint[0] += eps * bottomNormal[0];
  bottomPoint[1] += eps * bottomNormal[1];
  bottomPoint[2] += eps * bottomNormal[2];
  this->BottomCapSource->SetCenter(bottomPoint);
  this->BottomCapSource->SetNormal(bottomNormal);
  this->BottomCapSource->SetRadius(this->Radius);
}

void vtkBoreholeRepresentation::UpdateGlyphActors()
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

  const double glyphRadius = std::max(0.25 * this->Radius, 0.1);
  this->TopGlyphSource->SetCenter(topPoint);
  this->TopGlyphSource->SetRadius(glyphRadius);
  this->BottomGlyphSource->SetCenter(bottomPoint);
  this->BottomGlyphSource->SetRadius(glyphRadius);
}

bool vtkBoreholeRepresentation::PickWorldPoint(int X, int Y, double worldPt[3])
{
  return this->PickWorldPointFromProps(X, Y, this->WallActor, nullptr, worldPt, nullptr);
}

bool vtkBoreholeRepresentation::PickWorldPointFromProps(
  int X, int Y, vtkProp* first, vtkProp* second, double worldPt[3], vtkProp** pickedProp)
{
  if (!this->Renderer)
  {
    return false;
  }

  this->Picker->InitializePickList();
  if (first)
  {
    this->Picker->AddPickList(first);
  }
  if (second)
  {
    this->Picker->AddPickList(second);
  }

  if (!this->Picker->Pick(static_cast<double>(X), static_cast<double>(Y), 0.0, this->Renderer))
  {
    return false;
  }

  if (pickedProp)
  {
    *pickedProp = this->Picker->GetViewProp();
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

bool vtkBoreholeRepresentation::EvaluateSurfaceHeightAtXY(
  vtkPolyData* surface, double x, double y, double& z) const
{
  if (!surface || !surface->GetPoints())
  {
    return false;
  }

  double bounds[6];
  surface->GetBounds(bounds);
  if (x < bounds[0] || x > bounds[1] || y < bounds[2] || y > bounds[3])
  {
    return false;
  }

  vtkPoints* pts = surface->GetPoints();
  vtkIdType bestId = -1;
  double bestD2 = VTK_DOUBLE_MAX;
  for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i)
  {
    double p[3];
    pts->GetPoint(i, p);
    const double dx = p[0] - x;
    const double dy = p[1] - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < bestD2)
    {
      bestD2 = d2;
      bestId = i;
    }
  }

  if (bestId < 0)
  {
    return false;
  }

  double p[3];
  pts->GetPoint(bestId, p);
  z = p[2];
  return true;
}

bool vtkBoreholeRepresentation::ComputeInterpolatedSurfaceNormal(
  const double point[3], double normal[3]) const
{
  if (!this->StructuralSurfaces || this->StructuralSurfaces->GetNumberOfItems() < 1)
  {
    return false;
  }

  struct Sample
  {
    vtkPolyData* Surface = nullptr;
    double Z = 0.0;
  };
  std::vector<Sample> samples;

  this->StructuralSurfaces->InitTraversal();
  while (vtkPolyData* surface = vtkPolyData::SafeDownCast(this->StructuralSurfaces->GetNextItemAsObject()))
  {
    double z = 0.0;
    if (this->EvaluateSurfaceHeightAtXY(surface, point[0], point[1], z))
    {
      samples.push_back({ surface, z });
    }
  }

  if (samples.empty())
  {
    return false;
  }
  std::sort(samples.begin(), samples.end(),
    [](const Sample& a, const Sample& b) { return a.Z < b.Z; });

  const Sample* low = &samples.front();
  const Sample* high = &samples.back();
  for (size_t i = 1; i < samples.size(); ++i)
  {
    if (point[2] <= samples[i].Z)
    {
      low = &samples[i - 1];
      high = &samples[i];
      break;
    }
  }

  const double denom = std::max(high->Z - low->Z, 1e-12);
  const double a = std::clamp((point[2] - low->Z) / denom, 0.0, 1.0);

  auto evalInterpZ = [&](double x, double y, double& zInterp) -> bool {
    double zLow = 0.0;
    double zHigh = 0.0;
    if (!this->EvaluateSurfaceHeightAtXY(low->Surface, x, y, zLow))
    {
      return false;
    }
    if (!this->EvaluateSurfaceHeightAtXY(high->Surface, x, y, zHigh))
    {
      return false;
    }
    zInterp = (1.0 - a) * zLow + a * zHigh;
    return true;
  };

  double zpx = 0.0, zmx = 0.0, zpy = 0.0, zmy = 0.0;
  if (!evalInterpZ(point[0] + this->SurfaceDx, point[1], zpx) ||
    !evalInterpZ(point[0] - this->SurfaceDx, point[1], zmx) ||
    !evalInterpZ(point[0], point[1] + this->SurfaceDy, zpy) ||
    !evalInterpZ(point[0], point[1] - this->SurfaceDy, zmy))
  {
    return false;
  }

  const double dzdx = (zpx - zmx) / (2.0 * this->SurfaceDx);
  const double dzdy = (zpy - zmy) / (2.0 * this->SurfaceDy);
  normal[0] = -dzdx;
  normal[1] = -dzdy;
  normal[2] = 1.0;
  vtkMath::Normalize(normal);
  return true;
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
  double worldPt[3];
  vtkProp* prop = nullptr;
  if (this->PickWorldPointFromProps(X, Y, this->TopGlyphActor, this->BottomGlyphActor, worldPt, &prop))
  {
    if (prop == this->TopGlyphActor)
    {
      state = OverTopGlyph;
    }
    else if (prop == this->BottomGlyphActor)
    {
      state = OverBottomGlyph;
    }
  }
  else if (this->PickWorldPointFromProps(X, Y, this->WallActor, nullptr, worldPt, &prop) &&
    prop == this->WallActor)
  {
    state = OverWall;
  }

  this->InteractionState = state;
  this->HighlightPart(state);
  return state;
}

void vtkBoreholeRepresentation::StartWidgetInteraction(double eventPos[2])
{
  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];
  this->DragInitialized = false;

  if (this->CurrentOperation == DragTopCap)
  {
    this->ActiveT = this->TopPosition;
  }
  else if (this->CurrentOperation == DragBottomCap)
  {
    this->ActiveT = this->BottomPosition;
  }

  double worldPt[3];
  const int x = static_cast<int>(eventPos[0]);
  const int y = static_cast<int>(eventPos[1]);
  bool picked = false;
  if (this->CurrentOperation == DragTopCap)
  {
    picked = this->PickWorldPointFromProps(x, y, this->AxisActor, nullptr, worldPt, nullptr);
  }
  else if (this->CurrentOperation == DragBottomCap)
  {
    picked = this->PickWorldPointFromProps(x, y, this->AxisActor, nullptr, worldPt, nullptr);
  }
  else if (this->CurrentOperation == DragWallRadius)
  {
    picked = this->PickWorldPointFromProps(x, y, this->WallActor, nullptr, worldPt, nullptr);
    if (picked)
    {
      double t = 0.0;
      double distance = 0.0;
      if (this->ComputeClosestOnTrajectory(worldPt, t, distance))
      {
        this->ActiveT = t;
      }
    }
  }

  if (picked)
  {
    this->LastPickPosition[0] = worldPt[0];
    this->LastPickPosition[1] = worldPt[1];
    this->LastPickPosition[2] = worldPt[2];
    this->DragInitialized = true;
  }
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
  const int x = static_cast<int>(newEventPos[0]);
  const int y = static_cast<int>(newEventPos[1]);
  bool picked = false;
  if (this->CurrentOperation == DragTopCap)
  {
    picked = this->PickWorldPointFromProps(x, y, this->AxisActor, nullptr, worldPt, nullptr);
  }
  else if (this->CurrentOperation == DragBottomCap)
  {
    picked = this->PickWorldPointFromProps(x, y, this->AxisActor, nullptr, worldPt, nullptr);
  }
  else if (this->CurrentOperation == DragWallRadius)
  {
    picked = this->PickWorldPointFromProps(x, y, this->WallActor, nullptr, worldPt, nullptr);
  }

  if (!picked)
  {
    return;
  }

  if (!this->DragInitialized)
  {
    this->LastPickPosition[0] = worldPt[0];
    this->LastPickPosition[1] = worldPt[1];
    this->LastPickPosition[2] = worldPt[2];
    this->DragInitialized = true;
    return;
  }

  const double minGap = 1e-4;
  if (this->CurrentOperation == DragTopCap)
  {
    double t = 0.0;
    double distance = 0.0;
    if (this->ComputeClosestOnTrajectory(worldPt, t, distance))
    {
      this->ActiveT = std::clamp(t, 0.0, this->BottomPosition - minGap);
      this->TopPosition = this->ActiveT;
    }
  }
  else if (this->CurrentOperation == DragBottomCap)
  {
    double t = 0.0;
    double distance = 0.0;
    if (this->ComputeClosestOnTrajectory(worldPt, t, distance))
    {
      this->ActiveT = std::clamp(t, this->TopPosition + minGap, 1.0);
      this->BottomPosition = this->ActiveT;
    }
  }
  else if (this->CurrentOperation == DragWallRadius)
  {
    double center[3];
    double tangent[3];
    if (this->ComputePointAndTangent(this->ActiveT, center, tangent))
    {
      double displayCenter[3];
      this->Renderer->SetWorldPoint(center[0], center[1], center[2], 1.0);
      this->Renderer->WorldToDisplay();
      this->Renderer->GetDisplayPoint(displayCenter);

      this->Renderer->SetDisplayPoint(newEventPos[0], newEventPos[1], displayCenter[2]);
      this->Renderer->DisplayToWorld();
      double world4[4];
      this->Renderer->GetWorldPoint(world4);
      if (std::abs(world4[3]) > 1e-12)
      {
        const double worldCursor[3] = { world4[0] / world4[3], world4[1] / world4[3], world4[2] / world4[3] };
        this->Radius = std::max(std::sqrt(vtkMath::Distance2BetweenPoints(worldCursor, center)), 1e-6);
      }
    }
  }

  this->LastPickPosition[0] = worldPt[0];
  this->LastPickPosition[1] = worldPt[1];
  this->LastPickPosition[2] = worldPt[2];

  this->BuildRepresentation();
  this->NeedToRenderOn();
}

void vtkBoreholeRepresentation::EndWidgetInteraction(double vtkNotUsed(newEventPos)[2])
{
  this->CurrentOperation = DragNone;
  this->DragInitialized = false;
}

void vtkBoreholeRepresentation::HighlightPart(int state)
{
  this->WallActor->SetProperty(state == OverWall ? this->SelectedWallProperty : this->DefaultWallProperty);
  this->TopGlyphActor->SetProperty(
    state == OverTopGlyph ? this->SelectedGlyphProperty : this->DefaultGlyphProperty);
  this->BottomGlyphActor->SetProperty(
    state == OverBottomGlyph ? this->SelectedGlyphProperty : this->DefaultGlyphProperty);
}

double* vtkBoreholeRepresentation::GetBounds()
{
  this->BuildRepresentation();
  return this->WallActor->GetBounds();
}

void vtkBoreholeRepresentation::GetActors(vtkPropCollection* pc)
{
  pc->AddItem(this->AxisActor);
  pc->AddItem(this->WallActor);
  pc->AddItem(this->TopCapActor);
  pc->AddItem(this->BottomCapActor);
  pc->AddItem(this->TopGlyphActor);
  pc->AddItem(this->BottomGlyphActor);
}

void vtkBoreholeRepresentation::ReleaseGraphicsResources(vtkWindow* w)
{
  this->AxisActor->ReleaseGraphicsResources(w);
  this->WallActor->ReleaseGraphicsResources(w);
  this->TopCapActor->ReleaseGraphicsResources(w);
  this->BottomCapActor->ReleaseGraphicsResources(w);
  this->TopGlyphActor->ReleaseGraphicsResources(w);
  this->BottomGlyphActor->ReleaseGraphicsResources(w);
}

int vtkBoreholeRepresentation::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->BuildRepresentation();
  int count = 0;
  count += this->AxisActor->RenderOpaqueGeometry(viewport);
  count += this->WallActor->RenderOpaqueGeometry(viewport);
  count += this->TopCapActor->RenderOpaqueGeometry(viewport);
  count += this->BottomCapActor->RenderOpaqueGeometry(viewport);
  count += this->TopGlyphActor->RenderOpaqueGeometry(viewport);
  count += this->BottomGlyphActor->RenderOpaqueGeometry(viewport);
  return count;
}

int vtkBoreholeRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* viewport)
{
  int count = 0;
  count += this->AxisActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->WallActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->TopCapActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->BottomCapActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->TopGlyphActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->BottomGlyphActor->RenderTranslucentPolygonalGeometry(viewport);
  return count;
}

vtkTypeBool vtkBoreholeRepresentation::HasTranslucentPolygonalGeometry()
{
  return this->AxisActor->HasTranslucentPolygonalGeometry() ||
    this->WallActor->HasTranslucentPolygonalGeometry() ||
    this->TopCapActor->HasTranslucentPolygonalGeometry() ||
    this->BottomCapActor->HasTranslucentPolygonalGeometry() ||
    this->TopGlyphActor->HasTranslucentPolygonalGeometry() ||
    this->BottomGlyphActor->HasTranslucentPolygonalGeometry();
}

void vtkBoreholeRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Radius: " << this->Radius << "\n";
  os << indent << "TopPosition: " << this->TopPosition << "\n";
  os << indent << "BottomPosition: " << this->BottomPosition << "\n";
}
VTK_ABI_NAMESPACE_END
