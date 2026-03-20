// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkStructuralSurfaceBoxRepresentation
 * @brief   box-like ROI representation constrained by a structural surface
 *
 * This representation defines an axis-aligned XY footprint whose top and
 * bottom faces are both sampled by interpolating between two structural
 * surfaces (2.5D polydata horizons). The ROI remains constrained in XY while
 * its vertical extent moves smoothly through the interval defined by the two
 * limiting surfaces.
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
class vtkPoints;
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
  vtkSetClampMacro(SamplingResolutionZ, int, 1, VTK_INT_MAX);
  vtkGetMacro(SamplingResolutionZ, int);

  // Convenience API that lets callers specify the number of sample points
  // instead of the number of intervals along each axis.
  void SetSamplingDimensions(int xPoints, int yPoints, int zPoints);
  bool GetSamplingDimensions(int dims[3]) const;

  vtkSetClampMacro(TopInterpolation, double, 0.0, 1.0);
  vtkGetMacro(TopInterpolation, double);
  vtkSetClampMacro(BottomInterpolation, double, 0.0, 1.0);
  vtkGetMacro(BottomInterpolation, double);
  // Handle radius is expressed in world coordinates and stays constant while
  // the ROI footprint and height change.
  vtkSetClampMacro(HandleRadius, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(HandleRadius, double);

  void SetFootprint(double xmin, double xmax, double ymin, double ymax);

  bool GetFootprint(double footprint[4]) const;
  bool EvaluateSurfaceHeight(double x, double y, double& z);
  // Returns the current closed-shell polydata that is rebuilt as the widget
  // moves. Callers can reuse its points/cells for custom scalar computation.
  vtkPolyData* GetClosedSurface();

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
  bool ComputeDisplayPoint(const double worldPt[3], double displayPt[3]);
  bool ComputeWorldPointOnHorizontalPlane(int X, int Y, double referenceZ, double worldPt[3]);
  bool ComputeWorldPointOnVerticalResizePlane(int X, int Y, const double anchor[3], double worldPt[3]);
  bool ComputeDisplayInterpolationParameter(int X, int Y, double& interpolation);
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
  int SamplingResolutionZ;
  double TopInterpolation;
  double BottomInterpolation;
  double HandleRadius;
  double InteractionInterpolationOffset;
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
