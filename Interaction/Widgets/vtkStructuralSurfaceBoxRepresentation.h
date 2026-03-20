// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkStructuralSurfaceBoxRepresentation
 * @brief   box-like ROI representation constrained by a structural surface
 *
 * This representation defines an axis-aligned XY footprint whose top face is
 * sampled from a selected structural surface (a 2.5D polydata surface). The
 * bottom face stays planar, allowing the region height to be adjusted while the
 * top face continuously follows the geological horizon.
 */

#ifndef vtkStructuralSurfaceBoxRepresentation_h
#define vtkStructuralSurfaceBoxRepresentation_h

#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWidgetRepresentation.h"
#include "vtkWrappingHints.h" // For VTK_MARSHALAUTO

#include <array>
#include <vector>

VTK_ABI_NAMESPACE_BEGIN
class vtkActor;
class vtkArrowSource;
class vtkCellLocator;
class vtkCellPicker;
class vtkPolyData;
class vtkPolyDataCollection;
class vtkPolyDataMapper;
class vtkPropCollection;
class vtkProperty;
class vtkTransformPolyDataFilter;
class vtkTriangleFilter;
class vtkViewport;
class vtkWindow;

class VTKINTERACTIONWIDGETS_EXPORT VTK_MARSHALAUTO vtkStructuralSurfaceBoxRepresentation
  : public vtkWidgetRepresentation
{
public:
  static vtkStructuralSurfaceBoxRepresentation* New();
  vtkTypeMacro(vtkStructuralSurfaceBoxRepresentation, vtkWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  enum InteractionStateType
  {
    Outside = 0,
    AdjustXMin,
    AdjustXMax,
    AdjustYMin,
    AdjustYMax,
    AdjustBottom,
    Translating
  };

  void SetStructuralSurfaces(vtkPolyDataCollection* surfaces);
  vtkGetObjectMacro(StructuralSurfaces, vtkPolyDataCollection);

  vtkSetClampMacro(ActiveSurfaceIndex, int, 0, VTK_INT_MAX);
  vtkGetMacro(ActiveSurfaceIndex, int);

  vtkSetClampMacro(SamplingResolutionX, int, 1, VTK_INT_MAX);
  vtkGetMacro(SamplingResolutionX, int);
  vtkSetClampMacro(SamplingResolutionY, int, 1, VTK_INT_MAX);
  vtkGetMacro(SamplingResolutionY, int);

  void SetFootprint(double xmin, double xmax, double ymin, double ymax);
  void SetBottomZ(double value);
  vtkGetMacro(BottomZ, double);

  bool GetFootprint(double footprint[4]) const;
  bool EvaluateSurfaceHeight(double x, double y, double& z);

  void BuildRepresentation() override;
  int ComputeInteractionState(int X, int Y, int modify = 0) override;
  void StartWidgetInteraction(double eventPos[2]) override;
  void WidgetInteraction(double eventPos[2]) override;
  void EndWidgetInteraction(double eventPos[2]) override;
  double* GetBounds() override;

  void GetActors(vtkPropCollection* pc) override;
  void ReleaseGraphicsResources(vtkWindow* w) override;
  int RenderOpaqueGeometry(vtkViewport* viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport* viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

protected:
  vtkStructuralSurfaceBoxRepresentation();
  ~vtkStructuralSurfaceBoxRepresentation() override;

  vtkPolyData* GetActiveSurface() const;
  bool EnsureActiveSurfaceLocator();
  bool ComputeWorldPointOnDisplayRay(int X, int Y, double displayZ, double worldPt[3]);
  bool ComputeWorldPointOnHorizontalPlane(int X, int Y, double referenceZ, double worldPt[3]);
  bool ComputeWorldPointOnVerticalResizePlane(int X, int Y, const double anchor[3], double worldPt[3]);
  bool IsPointInsideSurfacePerimeter(double x, double y) const;
  bool IsFootprintInsideSurfacePerimeter(const double footprint[4]) const;
  void ApplyConstrainedFootprint(const double proposed[4]);
  void HighlightPart(int state);
  void UpdateHandleGeometry(const double bounds[6]);
  double ComputeReferenceTopZ(double x, double y);
  double ComputeMinimumTopZForFootprint(const double footprint[4]);
  void GetHandleAnchorPoint(int handleId, double point[3]);
  void GetHandleDirection(int handleId, double direction[3]);
  void ClampBottomToSurface();
  void RebuildBoundaryLoop();

  vtkPolyDataCollection* StructuralSurfaces;
  int ActiveSurfaceIndex;
  int SamplingResolutionX;
  int SamplingResolutionY;
  double Footprint[4];
  double BottomZ;
  double LastPickPosition[3];
  double LastEventPosition[2];
  double Bounds[6];

  vtkTriangleFilter* SurfaceTriangulator;
  vtkCellLocator* SurfaceLocator;
  vtkPolyData* TriangulatedSurface;
  vtkTimeStamp LocatorBuildTime;
  vtkTimeStamp RepresentationBuildTime;

  vtkPolyData* SurfacePolyData;
  vtkPolyDataMapper* SurfaceMapper;
  vtkActor* SurfaceActor;
  vtkPolyData* OutlinePolyData;
  vtkPolyDataMapper* OutlineMapper;
  vtkActor* OutlineActor;

  vtkArrowSource* HandleSource;
  vtkTransformPolyDataFilter** HandleTransformFilters;
  vtkPolyDataMapper** HandleMappers;
  vtkActor** Handles;
  vtkCellPicker* Picker;

  vtkProperty* SurfaceProperty;
  vtkProperty* SelectedSurfaceProperty;
  vtkProperty* OutlineProperty;
  vtkProperty* SelectedOutlineProperty;
  vtkProperty* HandleProperty;
  vtkProperty* SelectedHandleProperty;

  std::vector<std::array<double, 2>> BoundaryLoop;
  double SurfaceXYBounds[4];

private:
  vtkStructuralSurfaceBoxRepresentation(const vtkStructuralSurfaceBoxRepresentation&) = delete;
  void operator=(const vtkStructuralSurfaceBoxRepresentation&) = delete;
};

VTK_ABI_NAMESPACE_END
#endif
