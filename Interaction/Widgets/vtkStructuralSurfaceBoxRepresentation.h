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
class vtkCellLocator;
class vtkCellPicker;
class vtkPolyData;
class vtkPolyDataCollection;
class vtkPolyDataMapper;
class vtkPropCollection;
class vtkProperty;
class vtkSphereSource;
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
    AdjustTop,
    AdjustBottom,
    Translating
  };

  void SetStructuralSurfaces(vtkPolyDataCollection* surfaces);
  vtkGetObjectMacro(StructuralSurfaces, vtkPolyDataCollection);

  vtkSetClampMacro(ActiveSurfaceIndex, int, 0, VTK_INT_MAX);
  vtkGetMacro(ActiveSurfaceIndex, int);
  vtkSetClampMacro(LowerSurfaceIndex, int, 0, VTK_INT_MAX);
  vtkGetMacro(LowerSurfaceIndex, int);

  vtkSetClampMacro(SamplingResolutionX, int, 1, VTK_INT_MAX);
  vtkGetMacro(SamplingResolutionX, int);
  vtkSetClampMacro(SamplingResolutionY, int, 1, VTK_INT_MAX);
  vtkGetMacro(SamplingResolutionY, int);

  vtkSetClampMacro(TopInterpolation, double, 0.0, 1.0);
  vtkGetMacro(TopInterpolation, double);
  vtkSetClampMacro(BottomInterpolation, double, 0.0, 1.0);
  vtkGetMacro(BottomInterpolation, double);

  void SetFootprint(double xmin, double xmax, double ymin, double ymax);

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
  vtkPolyData* GetLowerSurface() const;
  bool EnsureActiveSurfaceLocator();
  bool ComputeWorldPointOnDisplayRay(int X, int Y, double displayZ, double worldPt[3]);
  bool ComputeWorldPointOnHorizontalPlane(int X, int Y, double referenceZ, double worldPt[3]);
  bool ComputeWorldPointOnVerticalResizePlane(int X, int Y, const double anchor[3], double worldPt[3]);
  bool EvaluateSurfaceInterval(double x, double y, double& upperZ, double& lowerZ);
  bool EvaluateInterpolatedHeight(double x, double y, double interpolation, double& z);
  bool IsPointInsideSurfacePerimeter(double x, double y) const;
  bool IsFootprintInsideSurfacePerimeter(const double footprint[4]) const;
  void ApplyConstrainedFootprint(const double proposed[4]);
  void HighlightPart(int state);
  void UpdateHandleGeometry(const double bounds[6]);
  double ComputeInterpolatedReferenceZ(double x, double y, double interpolation);
  double ComputeMinimumInterpolatedGapForFootprint(const double footprint[4]);
  void GetHandleAnchorPoint(int handleId, double point[3]);
  void ClampInterpolationsToSurfaceInterval();
  void RebuildBoundaryLoop();

  vtkPolyDataCollection* StructuralSurfaces;
  int ActiveSurfaceIndex;
  int LowerSurfaceIndex;
  int SamplingResolutionX;
  int SamplingResolutionY;
  double TopInterpolation;
  double BottomInterpolation;
  double Footprint[4];
  double LastPickPosition[3];
  double LastEventPosition[2];
  double Bounds[6];

  vtkTriangleFilter* SurfaceTriangulator;
  vtkCellLocator* SurfaceLocator;
  vtkPolyData* TriangulatedSurface;
  vtkTriangleFilter* LowerSurfaceTriangulator;
  vtkCellLocator* LowerSurfaceLocator;
  vtkPolyData* LowerTriangulatedSurface;
  vtkTimeStamp LocatorBuildTime;
  vtkTimeStamp RepresentationBuildTime;

  vtkPolyData* SurfacePolyData;
  vtkPolyDataMapper* SurfaceMapper;
  vtkActor* SurfaceActor;
  vtkPolyData* OutlinePolyData;
  vtkPolyDataMapper* OutlineMapper;
  vtkActor* OutlineActor;

  vtkSphereSource** HandleSources;
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
