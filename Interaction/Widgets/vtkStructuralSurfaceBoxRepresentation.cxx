// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

#include "vtkStructuralSurfaceBoxRepresentation.h"

#include "vtkActor.h"
#include "vtkArrowSource.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkCellLocator.h"
#include "vtkCellPicker.h"
#include "vtkFeatureEdges.h"
#include "vtkMath.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataCollection.h"
#include "vtkPolyDataMapper.h"
#include "vtkPropCollection.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkStripper.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTriangleFilter.h"
#include "vtkViewport.h"
#include "vtkWindow.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkStructuralSurfaceBoxRepresentation);

namespace
{
constexpr int NumberOfHandles = 5;
constexpr double MinFootprintSize = 1e-3;
constexpr double MinHeight = 1e-3;
constexpr int BoundaryBinarySearchIterations = 24;

int InteractionStateFromHandle(int handleId)
{
  switch (handleId)
  {
    case 0:
      return vtkStructuralSurfaceBoxRepresentation::AdjustXMin;
    case 1:
      return vtkStructuralSurfaceBoxRepresentation::AdjustXMax;
    case 2:
      return vtkStructuralSurfaceBoxRepresentation::AdjustYMin;
    case 3:
      return vtkStructuralSurfaceBoxRepresentation::AdjustYMax;
    case 4:
      return vtkStructuralSurfaceBoxRepresentation::AdjustBottom;
    default:
      return vtkStructuralSurfaceBoxRepresentation::Outside;
  }
}

double SignedArea2D(const std::vector<std::array<double, 2>>& polygon)
{
  if (polygon.size() < 3)
  {
    return 0.0;
  }

  double area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i)
  {
    const auto& p0 = polygon[i];
    const auto& p1 = polygon[(i + 1) % polygon.size()];
    area += p0[0] * p1[1] - p1[0] * p0[1];
  }
  return 0.5 * area;
}

bool PointInPolygon2D(const std::vector<std::array<double, 2>>& polygon, double x, double y)
{
  if (polygon.size() < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    const auto& pi = polygon[i];
    const auto& pj = polygon[j];
    const bool intersects =
      ((pi[1] > y) != (pj[1] > y)) &&
      (x < (pj[0] - pi[0]) * (y - pi[1]) / std::max(pj[1] - pi[1], 1e-15) + pi[0]);
    if (intersects)
    {
      inside = !inside;
    }
  }
  return inside;
}
}

vtkStructuralSurfaceBoxRepresentation::vtkStructuralSurfaceBoxRepresentation()
  : StructuralSurfaces(nullptr)
  , ActiveSurfaceIndex(0)
  , SamplingResolutionX(12)
  , SamplingResolutionY(12)
  , BottomZ(-20.0)
{
  this->Footprint[0] = -10.0;
  this->Footprint[1] = 10.0;
  this->Footprint[2] = -10.0;
  this->Footprint[3] = 10.0;
  std::fill(this->LastPickPosition, this->LastPickPosition + 3, 0.0);
  std::fill(this->LastEventPosition, this->LastEventPosition + 2, 0.0);
  std::fill(this->Bounds, this->Bounds + 6, 0.0);
  std::fill(this->SurfaceXYBounds, this->SurfaceXYBounds + 4, 0.0);
  this->InteractionState = vtkStructuralSurfaceBoxRepresentation::Outside;

  this->SurfaceTriangulator = vtkTriangleFilter::New();
  this->SurfaceLocator = vtkCellLocator::New();
  this->TriangulatedSurface = vtkPolyData::New();

  this->SurfacePolyData = vtkPolyData::New();
  this->SurfaceMapper = vtkPolyDataMapper::New();
  this->SurfaceMapper->SetInputData(this->SurfacePolyData);
  this->SurfaceActor = vtkActor::New();
  this->SurfaceActor->SetMapper(this->SurfaceMapper);

  this->OutlinePolyData = vtkPolyData::New();
  this->OutlineMapper = vtkPolyDataMapper::New();
  this->OutlineMapper->SetInputData(this->OutlinePolyData);
  this->OutlineActor = vtkActor::New();
  this->OutlineActor->SetMapper(this->OutlineMapper);

  this->HandleSource = vtkArrowSource::New();
  this->HandleSource->SetShaftResolution(16);
  this->HandleSource->SetTipResolution(16);
  this->HandleSource->SetTipLength(0.30);
  this->HandleSource->SetTipRadius(0.12);
  this->HandleSource->SetShaftRadius(0.05);

  this->HandleTransformFilters = new vtkTransformPolyDataFilter*[NumberOfHandles];
  this->HandleMappers = new vtkPolyDataMapper*[NumberOfHandles];
  this->Handles = new vtkActor*[NumberOfHandles];
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->HandleTransformFilters[i] = vtkTransformPolyDataFilter::New();
    this->HandleTransformFilters[i]->SetInputConnection(this->HandleSource->GetOutputPort());
    this->HandleMappers[i] = vtkPolyDataMapper::New();
    this->HandleMappers[i]->SetInputConnection(this->HandleTransformFilters[i]->GetOutputPort());
    this->Handles[i] = vtkActor::New();
    this->Handles[i]->SetMapper(this->HandleMappers[i]);
  }

  this->SurfaceProperty = vtkProperty::New();
  this->SurfaceProperty->SetColor(0.55, 0.75, 0.95);
  this->SurfaceProperty->SetOpacity(0.24);
  this->SelectedSurfaceProperty = vtkProperty::New();
  this->SelectedSurfaceProperty->SetColor(0.95, 0.78, 0.28);
  this->SelectedSurfaceProperty->SetOpacity(0.30);
  this->OutlineProperty = vtkProperty::New();
  this->OutlineProperty->SetColor(1.0, 1.0, 1.0);
  this->OutlineProperty->SetLineWidth(2.2);
  this->SelectedOutlineProperty = vtkProperty::New();
  this->SelectedOutlineProperty->SetColor(1.0, 0.85, 0.25);
  this->SelectedOutlineProperty->SetLineWidth(2.6);
  this->HandleProperty = vtkProperty::New();
  this->HandleProperty->SetColor(0.35, 1.0, 0.35);
  this->SelectedHandleProperty = vtkProperty::New();
  this->SelectedHandleProperty->SetColor(1.0, 0.45, 0.15);

  this->SurfaceActor->SetProperty(this->SurfaceProperty);
  this->OutlineActor->SetProperty(this->OutlineProperty);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Handles[i]->SetProperty(this->HandleProperty);
  }

  this->Picker = vtkCellPicker::New();
  this->Picker->SetTolerance(0.005);
  this->Picker->PickFromListOn();
  this->Picker->AddPickList(this->SurfaceActor);
  this->Picker->AddPickList(this->OutlineActor);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Picker->AddPickList(this->Handles[i]);
  }
}

vtkStructuralSurfaceBoxRepresentation::~vtkStructuralSurfaceBoxRepresentation()
{
  this->SetStructuralSurfaces(nullptr);

  this->SurfaceTriangulator->Delete();
  this->SurfaceLocator->Delete();
  this->TriangulatedSurface->Delete();

  this->SurfacePolyData->Delete();
  this->SurfaceMapper->Delete();
  this->SurfaceActor->Delete();
  this->OutlinePolyData->Delete();
  this->OutlineMapper->Delete();
  this->OutlineActor->Delete();

  this->HandleSource->Delete();
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->HandleTransformFilters[i]->Delete();
    this->HandleMappers[i]->Delete();
    this->Handles[i]->Delete();
  }
  delete[] this->HandleTransformFilters;
  delete[] this->HandleMappers;
  delete[] this->Handles;

  this->Picker->Delete();

  this->SurfaceProperty->Delete();
  this->SelectedSurfaceProperty->Delete();
  this->OutlineProperty->Delete();
  this->SelectedOutlineProperty->Delete();
  this->HandleProperty->Delete();
  this->SelectedHandleProperty->Delete();
}

void vtkStructuralSurfaceBoxRepresentation::SetStructuralSurfaces(vtkPolyDataCollection* surfaces)
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

  this->BoundaryLoop.clear();
  this->LocatorBuildTime.Modified();
  this->Modified();
}

void vtkStructuralSurfaceBoxRepresentation::SetFootprint(
  double xmin, double xmax, double ymin, double ymax)
{
  if (xmin > xmax)
  {
    std::swap(xmin, xmax);
  }
  if (ymin > ymax)
  {
    std::swap(ymin, ymax);
  }

  double proposed[4] = { xmin, std::max(xmax, xmin + MinFootprintSize), ymin,
    std::max(ymax, ymin + MinFootprintSize) };

  this->ApplyConstrainedFootprint(proposed);
  this->Modified();
}

void vtkStructuralSurfaceBoxRepresentation::SetBottomZ(double value)
{
  if (this->BottomZ == value)
  {
    return;
  }
  this->BottomZ = value;
  this->ClampBottomToSurface();
  this->Modified();
}

bool vtkStructuralSurfaceBoxRepresentation::GetFootprint(double footprint[4]) const
{
  if (!footprint)
  {
    return false;
  }
  std::copy(this->Footprint, this->Footprint + 4, footprint);
  return true;
}

vtkPolyData* vtkStructuralSurfaceBoxRepresentation::GetActiveSurface() const
{
  if (!this->StructuralSurfaces)
  {
    return nullptr;
  }

  this->StructuralSurfaces->InitTraversal();
  vtkPolyData* surface = nullptr;
  for (int i = 0; i <= this->ActiveSurfaceIndex; ++i)
  {
    surface = this->StructuralSurfaces->GetNextItem();
    if (!surface)
    {
      break;
    }
  }
  return surface;
}

void vtkStructuralSurfaceBoxRepresentation::RebuildBoundaryLoop()
{
  this->BoundaryLoop.clear();
  if (!this->TriangulatedSurface || this->TriangulatedSurface->GetNumberOfCells() == 0)
  {
    return;
  }

  vtkNew<vtkFeatureEdges> featureEdges;
  featureEdges->BoundaryEdgesOn();
  featureEdges->FeatureEdgesOff();
  featureEdges->ManifoldEdgesOff();
  featureEdges->NonManifoldEdgesOff();
  featureEdges->SetInputData(this->TriangulatedSurface);
  featureEdges->Update();

  vtkNew<vtkStripper> stripper;
  stripper->JoinContiguousSegmentsOn();
  stripper->SetInputConnection(featureEdges->GetOutputPort());
  stripper->Update();

  vtkPolyData* loopPolyData = stripper->GetOutput();
  vtkCellArray* lines = loopPolyData->GetLines();
  if (!lines || !loopPolyData->GetPoints())
  {
    return;
  }

  vtkIdType npts = 0;
  const vtkIdType* pts = nullptr;
  double bestArea = 0.0;
  std::vector<std::array<double, 2>> bestLoop;
  for (lines->InitTraversal(); lines->GetNextCell(npts, pts);)
  {
    if (npts < 3)
    {
      continue;
    }

    std::vector<std::array<double, 2>> candidate;
    candidate.reserve(static_cast<std::size_t>(npts));
    for (vtkIdType i = 0; i < npts; ++i)
    {
      double p[3];
      loopPolyData->GetPoint(pts[i], p);
      candidate.push_back({ p[0], p[1] });
    }
    if (candidate.front() == candidate.back())
    {
      candidate.pop_back();
    }

    const double area = std::abs(SignedArea2D(candidate));
    if (area > bestArea)
    {
      bestArea = area;
      bestLoop = candidate;
    }
  }

  this->BoundaryLoop = bestLoop;
}

bool vtkStructuralSurfaceBoxRepresentation::EnsureActiveSurfaceLocator()
{
  vtkPolyData* activeSurface = this->GetActiveSurface();
  if (!activeSurface || !activeSurface->GetPoints() || activeSurface->GetNumberOfPoints() < 3)
  {
    return false;
  }

  if (this->LocatorBuildTime > activeSurface->GetMTime() &&
    this->LocatorBuildTime > this->GetMTime())
  {
    return this->TriangulatedSurface->GetNumberOfCells() > 0;
  }

  this->SurfaceTriangulator->SetInputData(activeSurface);
  this->SurfaceTriangulator->Update();
  this->TriangulatedSurface->DeepCopy(this->SurfaceTriangulator->GetOutput());
  this->SurfaceLocator->SetDataSet(this->TriangulatedSurface);
  this->SurfaceLocator->BuildLocator();

  double bounds[6];
  this->TriangulatedSurface->GetBounds(bounds);
  this->SurfaceXYBounds[0] = bounds[0];
  this->SurfaceXYBounds[1] = bounds[1];
  this->SurfaceXYBounds[2] = bounds[2];
  this->SurfaceXYBounds[3] = bounds[3];
  this->RebuildBoundaryLoop();

  this->LocatorBuildTime.Modified();
  return this->TriangulatedSurface->GetNumberOfCells() > 0;
}

bool vtkStructuralSurfaceBoxRepresentation::EvaluateSurfaceHeight(double x, double y, double& z)
{
  if (!this->EnsureActiveSurfaceLocator())
  {
    return false;
  }

  double bounds[6];
  this->TriangulatedSurface->GetBounds(bounds);
  double p0[3] = { x, y, bounds[4] - 1.0 };
  double p1[3] = { x, y, bounds[5] + 1.0 };
  double t = 0.0;
  double xyz[3];
  double pcoords[3];
  int subId = 0;
  vtkIdType cellId = -1;
  if (this->SurfaceLocator->IntersectWithLine(p0, p1, 1e-6, t, xyz, pcoords, subId, cellId))
  {
    z = xyz[2];
    return true;
  }

  return false;
}

bool vtkStructuralSurfaceBoxRepresentation::ComputeWorldPointOnDisplayRay(
  int X, int Y, double displayZ, double worldPt[3])
{
  if (!this->Renderer)
  {
    return false;
  }

  this->Renderer->SetDisplayPoint(static_cast<double>(X), static_cast<double>(Y), displayZ);
  this->Renderer->DisplayToWorld();
  double homogeneous[4];
  this->Renderer->GetWorldPoint(homogeneous);
  const double w = (std::abs(homogeneous[3]) > 1e-12 ? homogeneous[3] : 1.0);
  worldPt[0] = homogeneous[0] / w;
  worldPt[1] = homogeneous[1] / w;
  worldPt[2] = homogeneous[2] / w;
  return true;
}

bool vtkStructuralSurfaceBoxRepresentation::ComputeWorldPointOnHorizontalPlane(
  int X, int Y, double referenceZ, double worldPt[3])
{
  double p0[3];
  double p1[3];
  if (!this->ComputeWorldPointOnDisplayRay(X, Y, 0.0, p0) ||
    !this->ComputeWorldPointOnDisplayRay(X, Y, 1.0, p1))
  {
    return false;
  }

  const double dirZ = p1[2] - p0[2];
  if (std::abs(dirZ) < 1e-12)
  {
    return false;
  }

  const double t = (referenceZ - p0[2]) / dirZ;
  for (int i = 0; i < 3; ++i)
  {
    worldPt[i] = p0[i] + t * (p1[i] - p0[i]);
  }
  return true;
}

bool vtkStructuralSurfaceBoxRepresentation::ComputeWorldPointOnVerticalResizePlane(
  int X, int Y, const double anchor[3], double worldPt[3])
{
  if (!this->Renderer)
  {
    return false;
  }

  double p0[3];
  double p1[3];
  if (!this->ComputeWorldPointOnDisplayRay(X, Y, 0.0, p0) ||
    !this->ComputeWorldPointOnDisplayRay(X, Y, 1.0, p1))
  {
    return false;
  }

  double viewDirection[3] = { 0.0, 0.0, -1.0 };
  if (vtkCamera* camera = this->Renderer->GetActiveCamera())
  {
    camera->GetDirectionOfProjection(viewDirection);
    vtkMath::Normalize(viewDirection);
  }

  const double zAxis[3] = { 0.0, 0.0, 1.0 };
  double planeNormal[3];
  vtkMath::Cross(zAxis, viewDirection, planeNormal);
  if (vtkMath::Norm(planeNormal) < 1e-12)
  {
    planeNormal[0] = 1.0;
    planeNormal[1] = 0.0;
    planeNormal[2] = 0.0;
  }
  vtkMath::Normalize(planeNormal);

  double rayDirection[3] = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
  const double denominator = vtkMath::Dot(planeNormal, rayDirection);
  if (std::abs(denominator) < 1e-12)
  {
    return false;
  }

  double diff[3] = { anchor[0] - p0[0], anchor[1] - p0[1], anchor[2] - p0[2] };
  const double t = vtkMath::Dot(planeNormal, diff) / denominator;
  for (int i = 0; i < 3; ++i)
  {
    worldPt[i] = p0[i] + t * rayDirection[i];
  }
  return true;
}

bool vtkStructuralSurfaceBoxRepresentation::IsPointInsideSurfacePerimeter(double x, double y) const
{
  if (!this->BoundaryLoop.empty())
  {
    return PointInPolygon2D(this->BoundaryLoop, x, y);
  }

  return x >= this->SurfaceXYBounds[0] && x <= this->SurfaceXYBounds[1] && y >= this->SurfaceXYBounds[2] &&
    y <= this->SurfaceXYBounds[3];
}

bool vtkStructuralSurfaceBoxRepresentation::IsFootprintInsideSurfacePerimeter(const double footprint[4]) const
{
  const double xmin = footprint[0];
  const double xmax = footprint[1];
  const double ymin = footprint[2];
  const double ymax = footprint[3];
  const double xmid = 0.5 * (xmin + xmax);
  const double ymid = 0.5 * (ymin + ymax);

  const std::array<std::array<double, 2>, 9> probes = { std::array<double, 2>{ xmin, ymin },
    std::array<double, 2>{ xmax, ymin }, std::array<double, 2>{ xmax, ymax },
    std::array<double, 2>{ xmin, ymax }, std::array<double, 2>{ xmid, ymin },
    std::array<double, 2>{ xmax, ymid }, std::array<double, 2>{ xmid, ymax },
    std::array<double, 2>{ xmin, ymid }, std::array<double, 2>{ xmid, ymid } };

  for (const auto& probe : probes)
  {
    if (!this->IsPointInsideSurfacePerimeter(probe[0], probe[1]))
    {
      return false;
    }
  }
  return true;
}

void vtkStructuralSurfaceBoxRepresentation::ApplyConstrainedFootprint(const double proposed[4])
{
  double clamped[4] = { proposed[0], proposed[1], proposed[2], proposed[3] };
  clamped[1] = std::max(clamped[1], clamped[0] + MinFootprintSize);
  clamped[3] = std::max(clamped[3], clamped[2] + MinFootprintSize);

  if (this->EnsureActiveSurfaceLocator() && !this->IsFootprintInsideSurfacePerimeter(clamped))
  {
    double current[4] = { this->Footprint[0], this->Footprint[1], this->Footprint[2], this->Footprint[3] };
    double lo = 0.0;
    double hi = 1.0;
    for (int iter = 0; iter < BoundaryBinarySearchIterations; ++iter)
    {
      const double mid = 0.5 * (lo + hi);
      double candidate[4];
      for (int i = 0; i < 4; ++i)
      {
        candidate[i] = current[i] + mid * (clamped[i] - current[i]);
      }
      candidate[1] = std::max(candidate[1], candidate[0] + MinFootprintSize);
      candidate[3] = std::max(candidate[3], candidate[2] + MinFootprintSize);
      if (this->IsFootprintInsideSurfacePerimeter(candidate))
      {
        lo = mid;
      }
      else
      {
        hi = mid;
      }
    }

    for (int i = 0; i < 4; ++i)
    {
      clamped[i] = current[i] + lo * (clamped[i] - current[i]);
    }
    clamped[1] = std::max(clamped[1], clamped[0] + MinFootprintSize);
    clamped[3] = std::max(clamped[3], clamped[2] + MinFootprintSize);
  }

  std::copy(clamped, clamped + 4, this->Footprint);
  this->ClampBottomToSurface();
}

double vtkStructuralSurfaceBoxRepresentation::ComputeReferenceTopZ(double x, double y)
{
  double z = this->BottomZ + 10.0;
  if (!this->EvaluateSurfaceHeight(x, y, z))
  {
    vtkPolyData* activeSurface = this->GetActiveSurface();
    if (activeSurface)
    {
      double bounds[6];
      activeSurface->GetBounds(bounds);
      z = bounds[5];
    }
  }
  return z;
}

double vtkStructuralSurfaceBoxRepresentation::ComputeMinimumTopZForFootprint(const double footprint[4])
{
  const int nx = std::max(1, this->SamplingResolutionX);
  const int ny = std::max(1, this->SamplingResolutionY);
  const double dx = (footprint[1] - footprint[0]) / static_cast<double>(nx);
  const double dy = (footprint[3] - footprint[2]) / static_cast<double>(ny);

  double minTopZ = std::numeric_limits<double>::infinity();
  for (int j = 0; j <= ny; ++j)
  {
    const double y = footprint[2] + static_cast<double>(j) * dy;
    for (int i = 0; i <= nx; ++i)
    {
      const double x = footprint[0] + static_cast<double>(i) * dx;
      double z = this->ComputeReferenceTopZ(x, y);
      minTopZ = std::min(minTopZ, z);
    }
  }

  return std::isfinite(minTopZ) ? minTopZ : (this->BottomZ + MinHeight);
}

void vtkStructuralSurfaceBoxRepresentation::ClampBottomToSurface()
{
  const double minTopZ = this->ComputeMinimumTopZForFootprint(this->Footprint);
  this->BottomZ = std::min(this->BottomZ, minTopZ - MinHeight);
}

void vtkStructuralSurfaceBoxRepresentation::GetHandleAnchorPoint(int handleId, double point[3])
{
  const double xmid = 0.5 * (this->Footprint[0] + this->Footprint[1]);
  const double ymid = 0.5 * (this->Footprint[2] + this->Footprint[3]);
  switch (handleId)
  {
    case 0:
      point[0] = this->Footprint[0];
      point[1] = ymid;
      point[2] = this->ComputeReferenceTopZ(point[0], point[1]);
      break;
    case 1:
      point[0] = this->Footprint[1];
      point[1] = ymid;
      point[2] = this->ComputeReferenceTopZ(point[0], point[1]);
      break;
    case 2:
      point[0] = xmid;
      point[1] = this->Footprint[2];
      point[2] = this->ComputeReferenceTopZ(point[0], point[1]);
      break;
    case 3:
      point[0] = xmid;
      point[1] = this->Footprint[3];
      point[2] = this->ComputeReferenceTopZ(point[0], point[1]);
      break;
    case 4:
    default:
      point[0] = xmid;
      point[1] = ymid;
      point[2] = this->BottomZ;
      break;
  }
}

void vtkStructuralSurfaceBoxRepresentation::GetHandleDirection(int handleId, double direction[3])
{
  direction[0] = 0.0;
  direction[1] = 0.0;
  direction[2] = 0.0;
  switch (handleId)
  {
    case 0:
      direction[0] = -1.0;
      break;
    case 1:
      direction[0] = 1.0;
      break;
    case 2:
      direction[1] = -1.0;
      break;
    case 3:
      direction[1] = 1.0;
      break;
    case 4:
      direction[2] = -1.0;
      break;
  }
}

void vtkStructuralSurfaceBoxRepresentation::UpdateHandleGeometry(const double bounds[6])
{
  const double dx = bounds[1] - bounds[0];
  const double dy = bounds[3] - bounds[2];
  const double dz = std::max(bounds[5] - bounds[4], MinHeight);
  const double length = 0.11 * std::sqrt(dx * dx + dy * dy + dz * dz);
  const double width = 0.035 * std::sqrt(dx * dx + dy * dy + dz * dz);

  for (int i = 0; i < NumberOfHandles; ++i)
  {
    double anchor[3];
    double direction[3];
    this->GetHandleAnchorPoint(i, anchor);
    this->GetHandleDirection(i, direction);

    vtkNew<vtkTransform> transform;
    transform->PostMultiply();
    transform->Translate(anchor);
    if (i == 0)
    {
      transform->RotateZ(180.0);
    }
    else if (i == 2)
    {
      transform->RotateZ(-90.0);
    }
    else if (i == 3)
    {
      transform->RotateZ(90.0);
    }
    else if (i == 4)
    {
      transform->RotateY(-90.0);
    }
    transform->Scale(length, width, width);
    transform->Translate(-0.5, 0.0, 0.0);

    this->HandleTransformFilters[i]->SetTransform(transform);
    this->HandleTransformFilters[i]->Update();
  }
}

void vtkStructuralSurfaceBoxRepresentation::BuildRepresentation()
{
  if (this->RepresentationBuildTime > this->GetMTime() &&
    this->RepresentationBuildTime > this->LocatorBuildTime)
  {
    return;
  }

  this->EnsureActiveSurfaceLocator();
  this->ClampBottomToSurface();

  const int nx = this->SamplingResolutionX;
  const int ny = this->SamplingResolutionY;
  const vtkIdType planeSize = static_cast<vtkIdType>(nx + 1) * static_cast<vtkIdType>(ny + 1);
  const double dx = (this->Footprint[1] - this->Footprint[0]) / static_cast<double>(nx);
  const double dy = (this->Footprint[3] - this->Footprint[2]) / static_cast<double>(ny);

  vtkNew<vtkPoints> surfacePoints;
  surfacePoints->SetNumberOfPoints(2 * planeSize);
  vtkNew<vtkCellArray> polys;

  double minTopZ = std::numeric_limits<double>::infinity();
  double maxTopZ = -std::numeric_limits<double>::infinity();

  for (int j = 0; j <= ny; ++j)
  {
    const double y = this->Footprint[2] + static_cast<double>(j) * dy;
    for (int i = 0; i <= nx; ++i)
    {
      const double x = this->Footprint[0] + static_cast<double>(i) * dx;
      const vtkIdType id = static_cast<vtkIdType>(j) * (nx + 1) + i;
      double z = this->ComputeReferenceTopZ(x, y);
      minTopZ = std::min(minTopZ, z);
      maxTopZ = std::max(maxTopZ, z);
      surfacePoints->SetPoint(id, x, y, z);
      surfacePoints->SetPoint(id + planeSize, x, y, this->BottomZ);
    }
  }

  if (this->BottomZ > minTopZ - MinHeight)
  {
    this->BottomZ = minTopZ - MinHeight;
    for (vtkIdType id = planeSize; id < 2 * planeSize; ++id)
    {
      double p[3];
      surfacePoints->GetPoint(id, p);
      p[2] = this->BottomZ;
      surfacePoints->SetPoint(id, p);
    }
  }

  for (int j = 0; j < ny; ++j)
  {
    for (int i = 0; i < nx; ++i)
    {
      const vtkIdType p00 = static_cast<vtkIdType>(j) * (nx + 1) + i;
      const vtkIdType p10 = p00 + 1;
      const vtkIdType p01 = static_cast<vtkIdType>(j + 1) * (nx + 1) + i;
      const vtkIdType p11 = p01 + 1;
      vtkIdType topTri0[3] = { p00, p10, p11 };
      vtkIdType topTri1[3] = { p00, p11, p01 };
      vtkIdType bottomTri0[3] = { p00 + planeSize, p11 + planeSize, p10 + planeSize };
      vtkIdType bottomTri1[3] = { p00 + planeSize, p01 + planeSize, p11 + planeSize };
      polys->InsertNextCell(3, topTri0);
      polys->InsertNextCell(3, topTri1);
      polys->InsertNextCell(3, bottomTri0);
      polys->InsertNextCell(3, bottomTri1);
    }
  }

  for (int i = 0; i < nx; ++i)
  {
    vtkIdType front0 = i;
    vtkIdType front1 = i + 1;
    vtkIdType frontQuad0[3] = { front0, front1, front1 + planeSize };
    vtkIdType frontQuad1[3] = { front0, front1 + planeSize, front0 + planeSize };
    polys->InsertNextCell(3, frontQuad0);
    polys->InsertNextCell(3, frontQuad1);

    vtkIdType back0 = static_cast<vtkIdType>(ny) * (nx + 1) + i;
    vtkIdType back1 = back0 + 1;
    vtkIdType backQuad0[3] = { back0, back0 + planeSize, back1 + planeSize };
    vtkIdType backQuad1[3] = { back0, back1 + planeSize, back1 };
    polys->InsertNextCell(3, backQuad0);
    polys->InsertNextCell(3, backQuad1);
  }

  for (int j = 0; j < ny; ++j)
  {
    vtkIdType left0 = static_cast<vtkIdType>(j) * (nx + 1);
    vtkIdType left1 = static_cast<vtkIdType>(j + 1) * (nx + 1);
    vtkIdType leftQuad0[3] = { left0, left0 + planeSize, left1 + planeSize };
    vtkIdType leftQuad1[3] = { left0, left1 + planeSize, left1 };
    polys->InsertNextCell(3, leftQuad0);
    polys->InsertNextCell(3, leftQuad1);

    vtkIdType right0 = static_cast<vtkIdType>(j) * (nx + 1) + nx;
    vtkIdType right1 = static_cast<vtkIdType>(j + 1) * (nx + 1) + nx;
    vtkIdType rightQuad0[3] = { right0, right1, right1 + planeSize };
    vtkIdType rightQuad1[3] = { right0, right1 + planeSize, right0 + planeSize };
    polys->InsertNextCell(3, rightQuad0);
    polys->InsertNextCell(3, rightQuad1);
  }

  this->SurfacePolyData->SetPoints(surfacePoints);
  this->SurfacePolyData->SetPolys(polys);
  this->SurfacePolyData->BuildCells();
  this->SurfacePolyData->ComputeBounds();
  this->SurfacePolyData->GetBounds(this->Bounds);

  vtkNew<vtkPoints> outlinePoints;
  vtkNew<vtkCellArray> outlineLines;
  outlinePoints->DeepCopy(surfacePoints);

  vtkNew<vtkIdList> topLoop;
  for (int i = 0; i <= nx; ++i)
  {
    topLoop->InsertNextId(i);
  }
  for (int j = 1; j <= ny; ++j)
  {
    topLoop->InsertNextId(static_cast<vtkIdType>(j) * (nx + 1) + nx);
  }
  for (int i = nx - 1; i >= 0; --i)
  {
    topLoop->InsertNextId(static_cast<vtkIdType>(ny) * (nx + 1) + i);
  }
  for (int j = ny - 1; j >= 1; --j)
  {
    topLoop->InsertNextId(static_cast<vtkIdType>(j) * (nx + 1));
  }
  topLoop->InsertNextId(0);
  outlineLines->InsertNextCell(topLoop);

  vtkIdType bottomLoop[5] = { planeSize, planeSize + nx, planeSize + static_cast<vtkIdType>(ny) * (nx + 1) + nx,
    planeSize + static_cast<vtkIdType>(ny) * (nx + 1), planeSize };
  outlineLines->InsertNextCell(5, bottomLoop);

  vtkIdType verticalEdge0[2] = { 0, planeSize };
  vtkIdType verticalEdge1[2] = { nx, planeSize + nx };
  vtkIdType verticalEdge2[2] = { static_cast<vtkIdType>(ny) * (nx + 1) + nx,
    planeSize + static_cast<vtkIdType>(ny) * (nx + 1) + nx };
  vtkIdType verticalEdge3[2] = { static_cast<vtkIdType>(ny) * (nx + 1),
    planeSize + static_cast<vtkIdType>(ny) * (nx + 1) };
  outlineLines->InsertNextCell(2, verticalEdge0);
  outlineLines->InsertNextCell(2, verticalEdge1);
  outlineLines->InsertNextCell(2, verticalEdge2);
  outlineLines->InsertNextCell(2, verticalEdge3);

  this->OutlinePolyData->SetPoints(outlinePoints);
  this->OutlinePolyData->SetLines(outlineLines);
  this->OutlinePolyData->ComputeBounds();

  this->UpdateHandleGeometry(this->Bounds);
  this->RepresentationBuildTime.Modified();
}

int vtkStructuralSurfaceBoxRepresentation::ComputeInteractionState(int X, int Y, int)
{
  this->InteractionState = vtkStructuralSurfaceBoxRepresentation::Outside;
  if (!this->Renderer)
  {
    return this->InteractionState;
  }

  this->BuildRepresentation();
  if (!this->Picker->Pick(X, Y, 0.0, this->Renderer))
  {
    this->HighlightPart(this->InteractionState);
    return this->InteractionState;
  }

  vtkProp* picked = this->Picker->GetViewProp();
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    if (picked == this->Handles[i])
    {
      this->InteractionState = InteractionStateFromHandle(i);
      this->HighlightPart(this->InteractionState);
      return this->InteractionState;
    }
  }

  if (picked == this->SurfaceActor || picked == this->OutlineActor)
  {
    this->InteractionState = Translating;
  }

  this->HighlightPart(this->InteractionState);
  return this->InteractionState;
}

void vtkStructuralSurfaceBoxRepresentation::StartWidgetInteraction(double eventPos[2])
{
  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];

  const double xmid = 0.5 * (this->Footprint[0] + this->Footprint[1]);
  const double ymid = 0.5 * (this->Footprint[2] + this->Footprint[3]);
  const double refZ = this->ComputeReferenceTopZ(xmid, ymid);

  if (this->InteractionState == AdjustBottom)
  {
    const double anchor[3] = { xmid, ymid, this->BottomZ };
    this->ComputeWorldPointOnVerticalResizePlane(
      static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), anchor, this->LastPickPosition);
  }
  else
  {
    this->ComputeWorldPointOnHorizontalPlane(
      static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), refZ, this->LastPickPosition);
  }
}

void vtkStructuralSurfaceBoxRepresentation::WidgetInteraction(double eventPos[2])
{
  const double xmid = 0.5 * (this->Footprint[0] + this->Footprint[1]);
  const double ymid = 0.5 * (this->Footprint[2] + this->Footprint[3]);
  const double refZ = this->ComputeReferenceTopZ(xmid, ymid);

  if (this->InteractionState == AdjustBottom)
  {
    double anchor[3] = { xmid, ymid, this->BottomZ };
    double worldPt[3];
    if (!this->ComputeWorldPointOnVerticalResizePlane(
          static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), anchor, worldPt))
    {
      return;
    }
    this->BottomZ = worldPt[2];
    this->ClampBottomToSurface();
    std::copy(worldPt, worldPt + 3, this->LastPickPosition);
  }
  else
  {
    double worldPt[3];
    if (!this->ComputeWorldPointOnHorizontalPlane(
          static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), refZ, worldPt))
    {
      return;
    }

    double proposed[4] = { this->Footprint[0], this->Footprint[1], this->Footprint[2], this->Footprint[3] };
    const double deltaX = worldPt[0] - this->LastPickPosition[0];
    const double deltaY = worldPt[1] - this->LastPickPosition[1];

    switch (this->InteractionState)
    {
      case AdjustXMin:
        proposed[0] = std::min(worldPt[0], proposed[1] - MinFootprintSize);
        break;
      case AdjustXMax:
        proposed[1] = std::max(worldPt[0], proposed[0] + MinFootprintSize);
        break;
      case AdjustYMin:
        proposed[2] = std::min(worldPt[1], proposed[3] - MinFootprintSize);
        break;
      case AdjustYMax:
        proposed[3] = std::max(worldPt[1], proposed[2] + MinFootprintSize);
        break;
      case Translating:
        proposed[0] += deltaX;
        proposed[1] += deltaX;
        proposed[2] += deltaY;
        proposed[3] += deltaY;
        break;
      case Outside:
      case AdjustBottom:
      default:
        break;
    }

    this->ApplyConstrainedFootprint(proposed);
    std::copy(worldPt, worldPt + 3, this->LastPickPosition);
  }

  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];
  this->Modified();
}

void vtkStructuralSurfaceBoxRepresentation::EndWidgetInteraction(double[2])
{
  this->BuildRepresentation();
}

double* vtkStructuralSurfaceBoxRepresentation::GetBounds()
{
  this->BuildRepresentation();
  return this->Bounds;
}

void vtkStructuralSurfaceBoxRepresentation::HighlightPart(int state)
{
  const bool active = state != vtkStructuralSurfaceBoxRepresentation::Outside;
  this->SurfaceActor->SetProperty(active ? this->SelectedSurfaceProperty : this->SurfaceProperty);
  this->OutlineActor->SetProperty(active ? this->SelectedOutlineProperty : this->OutlineProperty);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Handles[i]->SetProperty(this->HandleProperty);
  }

  int highlightedHandle = -1;
  switch (state)
  {
    case AdjustXMin:
      highlightedHandle = 0;
      break;
    case AdjustXMax:
      highlightedHandle = 1;
      break;
    case AdjustYMin:
      highlightedHandle = 2;
      break;
    case AdjustYMax:
      highlightedHandle = 3;
      break;
    case AdjustBottom:
      highlightedHandle = 4;
      break;
    case Outside:
    case Translating:
    default:
      break;
  }

  if (highlightedHandle >= 0)
  {
    this->Handles[highlightedHandle]->SetProperty(this->SelectedHandleProperty);
  }
}

void vtkStructuralSurfaceBoxRepresentation::GetActors(vtkPropCollection* pc)
{
  pc->AddItem(this->SurfaceActor);
  pc->AddItem(this->OutlineActor);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    pc->AddItem(this->Handles[i]);
  }
}

void vtkStructuralSurfaceBoxRepresentation::ReleaseGraphicsResources(vtkWindow* w)
{
  this->SurfaceActor->ReleaseGraphicsResources(w);
  this->OutlineActor->ReleaseGraphicsResources(w);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Handles[i]->ReleaseGraphicsResources(w);
  }
}

int vtkStructuralSurfaceBoxRepresentation::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->BuildRepresentation();
  int count = this->SurfaceActor->RenderOpaqueGeometry(viewport);
  count += this->OutlineActor->RenderOpaqueGeometry(viewport);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    count += this->Handles[i]->RenderOpaqueGeometry(viewport);
  }
  return count;
}

int vtkStructuralSurfaceBoxRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* viewport)
{
  this->BuildRepresentation();
  int count = this->SurfaceActor->RenderTranslucentPolygonalGeometry(viewport);
  count += this->OutlineActor->RenderTranslucentPolygonalGeometry(viewport);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    count += this->Handles[i]->RenderTranslucentPolygonalGeometry(viewport);
  }
  return count;
}

vtkTypeBool vtkStructuralSurfaceBoxRepresentation::HasTranslucentPolygonalGeometry()
{
  return 1;
}

void vtkStructuralSurfaceBoxRepresentation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "ActiveSurfaceIndex: " << this->ActiveSurfaceIndex << "\n";
  os << indent << "SamplingResolutionX: " << this->SamplingResolutionX << "\n";
  os << indent << "SamplingResolutionY: " << this->SamplingResolutionY << "\n";
  os << indent << "Footprint: [" << this->Footprint[0] << ", " << this->Footprint[1] << ", "
     << this->Footprint[2] << ", " << this->Footprint[3] << "]\n";
  os << indent << "BottomZ: " << this->BottomZ << "\n";
}
VTK_ABI_NAMESPACE_END
