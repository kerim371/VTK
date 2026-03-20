// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

#include "vtkStructuralSurfaceBoxRepresentation.h"

#include "vtkActor.h"
#include "vtkCellArray.h"
#include "vtkCellLocator.h"
#include "vtkCellPicker.h"
#include "vtkMath.h"
#include "vtkNew.h"
#include "vtkObjectFactory.h"
#include "vtkPlane.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataCollection.h"
#include "vtkPolyDataMapper.h"
#include "vtkPropCollection.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkTriangleFilter.h"
#include "vtkViewport.h"
#include "vtkWindow.h"

#include <algorithm>
#include <array>
#include <cmath>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkStructuralSurfaceBoxRepresentation);

namespace
{
constexpr int NumberOfHandles = 6;
constexpr double MinFootprintSize = 1e-3;
constexpr double MinHeight = 1e-3;
}

vtkStructuralSurfaceBoxRepresentation::vtkStructuralSurfaceBoxRepresentation()
  : StructuralSurfaces(nullptr)
  , ActiveSurfaceIndex(0)
  , SamplingResolutionX(20)
  , SamplingResolutionY(20)
  , BottomZ(-20.0)
{
  this->Footprint[0] = -10.0;
  this->Footprint[1] = 10.0;
  this->Footprint[2] = -10.0;
  this->Footprint[3] = 10.0;
  std::fill(this->LastPickPosition, this->LastPickPosition + 3, 0.0);
  std::fill(this->LastEventPosition, this->LastEventPosition + 2, 0.0);
  std::fill(this->Bounds, this->Bounds + 6, 0.0);
  this->InteractionState = vtkStructuralSurfaceBoxRepresentation::Outside;

  this->SurfaceTriangulator = vtkTriangleFilter::New();
  this->SurfaceLocator = vtkCellLocator::New();
  this->TriangulatedSurface = vtkPolyData::New();

  this->RegionPolyData = vtkPolyData::New();
  this->RegionMapper = vtkPolyDataMapper::New();
  this->RegionMapper->SetInputData(this->RegionPolyData);
  this->RegionActor = vtkActor::New();
  this->RegionActor->SetMapper(this->RegionMapper);

  this->OutlineMapper = vtkPolyDataMapper::New();
  this->OutlineMapper->SetInputData(this->RegionPolyData);
  this->OutlineActor = vtkActor::New();
  this->OutlineActor->SetMapper(this->OutlineMapper);

  this->HandleSources = new vtkSphereSource*[NumberOfHandles];
  this->HandleMappers = new vtkPolyDataMapper*[NumberOfHandles];
  this->Handles = new vtkActor*[NumberOfHandles];
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->HandleSources[i] = vtkSphereSource::New();
    this->HandleSources[i]->SetThetaResolution(20);
    this->HandleSources[i]->SetPhiResolution(20);
    this->HandleMappers[i] = vtkPolyDataMapper::New();
    this->HandleMappers[i]->SetInputConnection(this->HandleSources[i]->GetOutputPort());
    this->Handles[i] = vtkActor::New();
    this->Handles[i]->SetMapper(this->HandleMappers[i]);
  }

  this->RegionProperty = vtkProperty::New();
  this->RegionProperty->SetColor(0.6, 0.78, 0.95);
  this->RegionProperty->SetOpacity(0.30);
  this->SelectedRegionProperty = vtkProperty::New();
  this->SelectedRegionProperty->SetColor(0.95, 0.75, 0.25);
  this->SelectedRegionProperty->SetOpacity(0.40);
  this->OutlineProperty = vtkProperty::New();
  this->OutlineProperty->SetColor(0.9, 0.95, 1.0);
  this->OutlineProperty->SetRepresentationToWireframe();
  this->OutlineProperty->SetLineWidth(2.0);
  this->SelectedOutlineProperty = vtkProperty::New();
  this->SelectedOutlineProperty->DeepCopy(this->OutlineProperty);
  this->SelectedOutlineProperty->SetColor(1.0, 0.85, 0.2);
  this->SelectedHandleProperty = vtkProperty::New();
  this->SelectedHandleProperty->SetColor(1.0, 0.4, 0.1);
  this->HandleProperty = vtkProperty::New();
  this->HandleProperty->SetColor(0.25, 0.95, 0.35);

  this->RegionActor->SetProperty(this->RegionProperty);
  this->OutlineActor->SetProperty(this->OutlineProperty);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Handles[i]->SetProperty(this->HandleProperty);
  }

  this->Picker = vtkCellPicker::New();
  this->Picker->SetTolerance(0.005);
  this->Picker->PickFromListOn();
  this->Picker->AddPickList(this->RegionActor);
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

  this->RegionPolyData->Delete();
  this->RegionMapper->Delete();
  this->RegionActor->Delete();
  this->OutlineMapper->Delete();
  this->OutlineActor->Delete();

  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->HandleSources[i]->Delete();
    this->HandleMappers[i]->Delete();
    this->Handles[i]->Delete();
  }
  delete[] this->HandleSources;
  delete[] this->HandleMappers;
  delete[] this->Handles;

  this->Picker->Delete();

  this->RegionProperty->Delete();
  this->SelectedRegionProperty->Delete();
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
  xmax = std::max(xmax, xmin + MinFootprintSize);
  ymax = std::max(ymax, ymin + MinFootprintSize);

  if (this->Footprint[0] == xmin && this->Footprint[1] == xmax && this->Footprint[2] == ymin &&
    this->Footprint[3] == ymax)
  {
    return;
  }

  this->Footprint[0] = xmin;
  this->Footprint[1] = xmax;
  this->Footprint[2] = ymin;
  this->Footprint[3] = ymax;
  this->Modified();
}

void vtkStructuralSurfaceBoxRepresentation::SetBottomZ(double value)
{
  if (this->BottomZ == value)
  {
    return;
  }
  this->BottomZ = value;
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

void vtkStructuralSurfaceBoxRepresentation::GetHandlePosition(int handleId, double point[3])
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
      point[0] = xmid;
      point[1] = ymid;
      point[2] = this->BottomZ;
      break;
    case 5:
    default:
      point[0] = xmid;
      point[1] = ymid;
      point[2] = this->ComputeReferenceTopZ(point[0], point[1]);
      break;
  }
}

void vtkStructuralSurfaceBoxRepresentation::UpdateHandleGeometry(
  const double bounds[6], const double centerTop[3])
{
  const double dx = bounds[1] - bounds[0];
  const double dy = bounds[3] - bounds[2];
  const double dz = std::max(bounds[5] - bounds[4], MinHeight);
  const double radius = 0.03 * std::sqrt(dx * dx + dy * dy + dz * dz);

  for (int i = 0; i < NumberOfHandles; ++i)
  {
    double p[3];
    this->GetHandlePosition(i, p);
    if (i == 5)
    {
      p[0] = centerTop[0];
      p[1] = centerTop[1];
      p[2] = centerTop[2];
    }
    this->HandleSources[i]->SetCenter(p);
    this->HandleSources[i]->SetRadius(radius);
    this->HandleSources[i]->Update();
  }
}

void vtkStructuralSurfaceBoxRepresentation::BuildRepresentation()
{
  if (this->RepresentationBuildTime > this->GetMTime() &&
    this->RepresentationBuildTime > this->LocatorBuildTime)
  {
    return;
  }

  const int nx = this->SamplingResolutionX;
  const int ny = this->SamplingResolutionY;
  const vtkIdType planeSize = static_cast<vtkIdType>(nx + 1) * static_cast<vtkIdType>(ny + 1);

  vtkNew<vtkPoints> points;
  points->SetNumberOfPoints(2 * planeSize);
  vtkNew<vtkCellArray> polys;

  const double dx = (this->Footprint[1] - this->Footprint[0]) / static_cast<double>(nx);
  const double dy = (this->Footprint[3] - this->Footprint[2]) / static_cast<double>(ny);

  double minTopZ = VTK_DOUBLE_MAX;
  double maxTopZ = -VTK_DOUBLE_MAX;
  double centerTop[3] = { 0.5 * (this->Footprint[0] + this->Footprint[1]),
    0.5 * (this->Footprint[2] + this->Footprint[3]), 0.0 };

  for (int j = 0; j <= ny; ++j)
  {
    const double y = this->Footprint[2] + dy * static_cast<double>(j);
    for (int i = 0; i <= nx; ++i)
    {
      const double x = this->Footprint[0] + dx * static_cast<double>(i);
      double z = this->ComputeReferenceTopZ(x, y);
      minTopZ = std::min(minTopZ, z);
      maxTopZ = std::max(maxTopZ, z);
      const vtkIdType id = static_cast<vtkIdType>(j) * (nx + 1) + i;
      points->SetPoint(id, x, y, z);
      points->SetPoint(id + planeSize, x, y, this->BottomZ);
    }
  }

  this->EvaluateSurfaceHeight(centerTop[0], centerTop[1], centerTop[2]);
  if (centerTop[2] <= this->BottomZ + MinHeight)
  {
    centerTop[2] = maxTopZ;
  }

  if (this->BottomZ > minTopZ - MinHeight)
  {
    this->BottomZ = minTopZ - MinHeight;
    for (vtkIdType id = 0; id < planeSize; ++id)
    {
      double p[3];
      points->GetPoint(id + planeSize, p);
      p[2] = this->BottomZ;
      points->SetPoint(id + planeSize, p);
    }
  }

  for (int j = 0; j < ny; ++j)
  {
    for (int i = 0; i < nx; ++i)
    {
      vtkIdType ids[4] = {
        static_cast<vtkIdType>(j) * (nx + 1) + i,
        static_cast<vtkIdType>(j) * (nx + 1) + i + 1,
        static_cast<vtkIdType>(j + 1) * (nx + 1) + i + 1,
        static_cast<vtkIdType>(j + 1) * (nx + 1) + i };
      polys->InsertNextCell(4, ids);

      vtkIdType bottomIds[4] = { ids[0] + planeSize, ids[3] + planeSize, ids[2] + planeSize,
        ids[1] + planeSize };
      polys->InsertNextCell(4, bottomIds);
    }
  }

  for (int i = 0; i < nx; ++i)
  {
    vtkIdType front[4] = { i, i + 1, i + 1 + planeSize, i + planeSize };
    polys->InsertNextCell(4, front);

    vtkIdType backTop0 = static_cast<vtkIdType>(ny) * (nx + 1) + i;
    vtkIdType back[4] = { backTop0, backTop0 + planeSize, backTop0 + 1 + planeSize, backTop0 + 1 };
    polys->InsertNextCell(4, back);
  }

  for (int j = 0; j < ny; ++j)
  {
    vtkIdType left0 = static_cast<vtkIdType>(j) * (nx + 1);
    vtkIdType left1 = static_cast<vtkIdType>(j + 1) * (nx + 1);
    vtkIdType left[4] = { left0, left0 + planeSize, left1 + planeSize, left1 };
    polys->InsertNextCell(4, left);

    vtkIdType right0 = static_cast<vtkIdType>(j) * (nx + 1) + nx;
    vtkIdType right1 = static_cast<vtkIdType>(j + 1) * (nx + 1) + nx;
    vtkIdType right[4] = { right0, right1, right1 + planeSize, right0 + planeSize };
    polys->InsertNextCell(4, right);
  }

  this->RegionPolyData->SetPoints(points);
  this->RegionPolyData->SetPolys(polys);
  this->RegionPolyData->BuildCells();
  this->RegionPolyData->ComputeBounds();
  this->RegionPolyData->GetBounds(this->Bounds);

  this->UpdateHandleGeometry(this->Bounds, centerTop);
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
      switch (i)
      {
        case 0:
          this->InteractionState = AdjustXMin;
          break;
        case 1:
          this->InteractionState = AdjustXMax;
          break;
        case 2:
          this->InteractionState = AdjustYMin;
          break;
        case 3:
          this->InteractionState = AdjustYMax;
          break;
        case 4:
          this->InteractionState = AdjustBottom;
          break;
        case 5:
          this->InteractionState = Translating;
          break;
      }
      this->HighlightPart(this->InteractionState);
      return this->InteractionState;
    }
  }

  if (picked == this->RegionActor || picked == this->OutlineActor)
  {
    this->InteractionState = Translating;
  }

  this->HighlightPart(this->InteractionState);
  return this->InteractionState;
}

bool vtkStructuralSurfaceBoxRepresentation::ComputeWorldPointOnReferencePlane(
  int X, int Y, double referenceZ, double worldPt[3])
{
  if (!this->Renderer)
  {
    return false;
  }

  auto getWorld = [&](double displayZ, double out[3]) {
    this->Renderer->SetDisplayPoint(static_cast<double>(X), static_cast<double>(Y), displayZ);
    this->Renderer->DisplayToWorld();
    double homogeneous[4];
    this->Renderer->GetWorldPoint(homogeneous);
    const double w = (std::abs(homogeneous[3]) > 1e-12 ? homogeneous[3] : 1.0);
    out[0] = homogeneous[0] / w;
    out[1] = homogeneous[1] / w;
    out[2] = homogeneous[2] / w;
  };

  double p0[3];
  double p1[3];
  getWorld(0.0, p0);
  getWorld(1.0, p1);

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

void vtkStructuralSurfaceBoxRepresentation::StartWidgetInteraction(double eventPos[2])
{
  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];

  const double xmid = 0.5 * (this->Footprint[0] + this->Footprint[1]);
  const double ymid = 0.5 * (this->Footprint[2] + this->Footprint[3]);
  const double refZ = this->ComputeReferenceTopZ(xmid, ymid);
  this->ComputeWorldPointOnReferencePlane(
    static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), refZ, this->LastPickPosition);
}

void vtkStructuralSurfaceBoxRepresentation::WidgetInteraction(double eventPos[2])
{
  const double xmid = 0.5 * (this->Footprint[0] + this->Footprint[1]);
  const double ymid = 0.5 * (this->Footprint[2] + this->Footprint[3]);
  const double refZ = this->ComputeReferenceTopZ(xmid, ymid);

  double worldPt[3];
  if (!this->ComputeWorldPointOnReferencePlane(
        static_cast<int>(eventPos[0]), static_cast<int>(eventPos[1]), refZ, worldPt))
  {
    return;
  }

  const double deltaX = worldPt[0] - this->LastPickPosition[0];
  const double deltaY = worldPt[1] - this->LastPickPosition[1];

  switch (this->InteractionState)
  {
    case AdjustXMin:
      this->Footprint[0] = std::min(worldPt[0], this->Footprint[1] - MinFootprintSize);
      break;
    case AdjustXMax:
      this->Footprint[1] = std::max(worldPt[0], this->Footprint[0] + MinFootprintSize);
      break;
    case AdjustYMin:
      this->Footprint[2] = std::min(worldPt[1], this->Footprint[3] - MinFootprintSize);
      break;
    case AdjustYMax:
      this->Footprint[3] = std::max(worldPt[1], this->Footprint[2] + MinFootprintSize);
      break;
    case Translating:
      this->Footprint[0] += deltaX;
      this->Footprint[1] += deltaX;
      this->Footprint[2] += deltaY;
      this->Footprint[3] += deltaY;
      break;
    case AdjustBottom:
    {
      const double scale = std::max({ this->Footprint[1] - this->Footprint[0],
        this->Footprint[3] - this->Footprint[2], refZ - this->BottomZ, 1.0 }) / 250.0;
      this->BottomZ += (this->LastEventPosition[1] - eventPos[1]) * scale;
      this->BottomZ = std::min(this->BottomZ, refZ - MinHeight);
    }
    break;
    case Outside:
    default:
      break;
  }

  this->LastEventPosition[0] = eventPos[0];
  this->LastEventPosition[1] = eventPos[1];
  std::copy(worldPt, worldPt + 3, this->LastPickPosition);
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
  this->RegionActor->SetProperty(active ? this->SelectedRegionProperty : this->RegionProperty);
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
    case Translating:
      highlightedHandle = 5;
      break;
    case Outside:
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
  pc->AddItem(this->RegionActor);
  pc->AddItem(this->OutlineActor);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    pc->AddItem(this->Handles[i]);
  }
}

void vtkStructuralSurfaceBoxRepresentation::ReleaseGraphicsResources(vtkWindow* w)
{
  this->RegionActor->ReleaseGraphicsResources(w);
  this->OutlineActor->ReleaseGraphicsResources(w);
  for (int i = 0; i < NumberOfHandles; ++i)
  {
    this->Handles[i]->ReleaseGraphicsResources(w);
  }
}

int vtkStructuralSurfaceBoxRepresentation::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->BuildRepresentation();
  int count = this->RegionActor->RenderOpaqueGeometry(viewport);
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
  int count = this->RegionActor->RenderTranslucentPolygonalGeometry(viewport);
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
