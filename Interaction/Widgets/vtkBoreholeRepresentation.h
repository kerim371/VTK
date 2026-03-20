// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkBoreholeRepresentation
 * @brief   interactive representation of a tubular borehole interval
 *
 * This representation visualizes a borehole built from a polyline trajectory.
 * A custom surface filter is used to build a closed borehole shell around the
 * trajectory. The top and bottom ends are constrained by structural surfaces,
 * while glyphs remain available for direct-manipulation interaction.
 */

#ifndef vtkBoreholeRepresentation_h
#define vtkBoreholeRepresentation_h

#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWidgetRepresentation.h"
#include "vtkWrappingHints.h" // For VTK_MARSHALAUTO

#include <vector>

VTK_ABI_NAMESPACE_BEGIN
class vtkActor;
class vtkCellPicker;
class vtkPlane;
class vtkPolyData;
class vtkPolyDataCollection;
class vtkPolyDataMapper;
class vtkProp;
class vtkProperty;
class vtkSphereSource;
class vtkBoreholeSurfaceFilter;

class VTKINTERACTIONWIDGETS_EXPORT VTK_MARSHALAUTO vtkBoreholeRepresentation
  : public vtkWidgetRepresentation
{
public:
  static vtkBoreholeRepresentation* New();
  vtkTypeMacro(vtkBoreholeRepresentation, vtkWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  enum InteractionStateType
  {
    Outside = 0,
    OverWall,
    OverTopGlyph,
    OverBottomGlyph
  };

  enum DragOperationType
  {
    DragNone = 0,
    DragWallRadius,
    DragTopCap,
    DragBottomCap
  };

  void SetInputData(vtkPolyData* polyData);
  vtkGetObjectMacro(Input, vtkPolyData);

  vtkSetClampMacro(Radius, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Radius, double);

  vtkSetClampMacro(TopPosition, double, 0.0, 1.0);
  vtkGetMacro(TopPosition, double);
  vtkSetClampMacro(BottomPosition, double, 0.0, 1.0);
  vtkGetMacro(BottomPosition, double);

  void SetStructuralSurfaces(vtkPolyDataCollection* surfaces);
  vtkGetObjectMacro(StructuralSurfaces, vtkPolyDataCollection);

  vtkSetClampMacro(SurfaceDx, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(SurfaceDx, double);
  vtkSetClampMacro(SurfaceDy, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(SurfaceDy, double);

  vtkSetClampMacro(SurfaceDz, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(SurfaceDz, double);

  vtkSetClampMacro(GlyphRadius, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(GlyphRadius, double);

  void BuildRepresentation() override;

  int ComputeInteractionState(int X, int Y, int modify = 0) override;
  void StartWidgetInteraction(double eventPos[2]) override;
  void WidgetInteraction(double newEventPos[2]) override;
  void EndWidgetInteraction(double newEventPos[2]) override;

  void SetCurrentOperation(int op);
  vtkGetMacro(CurrentOperation, int);

  double* GetBounds() override;
  void GetActors(vtkPropCollection* pc) override;
  void ReleaseGraphicsResources(vtkWindow* w) override;
  int RenderOpaqueGeometry(vtkViewport* viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport* viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

protected:
  vtkBoreholeRepresentation();
  ~vtkBoreholeRepresentation() override;

  void HighlightPart(int state);
  bool PickWorldPoint(int X, int Y, double worldPt[3]);
  bool PickWorldPointFromProps(
    int X, int Y, vtkProp* first, vtkProp* second, double worldPt[3], vtkProp** pickedProp = nullptr);
  bool ComputeClosestOnTrajectory(const double worldPt[3], double& t, double& distance) const;
  bool ComputePointAndTangent(double t, double point[3], double tangent[3]) const;
  bool ComputeInterpolatedSurfaceNormal(const double point[3], double normal[3]) const;
  bool EvaluateSurfaceHeightAtXY(vtkPolyData* surface, double x, double y, double& z) const;
  void UpdateClippingPlanes();
  void UpdateCapActors();
  void UpdateGlyphActors();

  vtkPolyData* Input;
  vtkPolyData* IntervalTrajectory;

  double Radius;
  double TopPosition;
  double BottomPosition;
  double GlyphRadius;

  vtkBoreholeSurfaceFilter* Tube;
  vtkPlane* TopPlane;
  vtkPlane* BottomPlane;
  vtkPolyDataCollection* StructuralSurfaces;

  vtkPolyDataMapper* WallMapper;
  vtkActor* WallActor;

  vtkPolyDataMapper* AxisMapper;
  vtkActor* AxisActor;
  vtkSphereSource* TopGlyphSource;
  vtkPolyDataMapper* TopGlyphMapper;
  vtkActor* TopGlyphActor;
  vtkSphereSource* BottomGlyphSource;
  vtkPolyDataMapper* BottomGlyphMapper;
  vtkActor* BottomGlyphActor;

  vtkProperty* DefaultWallProperty;
  vtkProperty* SelectedWallProperty;
  vtkProperty* DefaultCapProperty;
  vtkProperty* SelectedCapProperty;
  vtkProperty* AxisProperty;
  vtkProperty* DefaultGlyphProperty;
  vtkProperty* SelectedGlyphProperty;

  vtkCellPicker* Picker;

  std::vector<double> CumulativeLengths;
  double TotalLength;
  double LastEventPosition[2];
  double LastPickPosition[3];
  double ActiveT;
  vtkTypeBool DragInitialized;
  double SurfaceDx;
  double SurfaceDy;
  double SurfaceDz;

  int CurrentOperation;

private:
  vtkBoreholeRepresentation(const vtkBoreholeRepresentation&) = delete;
  void operator=(const vtkBoreholeRepresentation&) = delete;
};

VTK_ABI_NAMESPACE_END

#endif
