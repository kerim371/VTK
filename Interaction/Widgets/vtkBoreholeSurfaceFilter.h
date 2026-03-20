// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkBoreholeSurfaceFilter
 * @brief   build a closed borehole surface around a trajectory
 *
 * Generates a tube-like closed surface around an input polyline trajectory
 * without using vtkTubeFilter. The first and last rings are optionally
 * projected onto user-provided structural surfaces to form geological caps.
 */

#ifndef vtkBoreholeSurfaceFilter_h
#define vtkBoreholeSurfaceFilter_h

#include "vtkInteractionWidgetsModule.h"
#include "vtkPolyDataAlgorithm.h"

VTK_ABI_NAMESPACE_BEGIN
class vtkPolyData;

class VTKINTERACTIONWIDGETS_EXPORT vtkBoreholeSurfaceFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkBoreholeSurfaceFilter* New();
  vtkTypeMacro(vtkBoreholeSurfaceFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  vtkSetClampMacro(Radius, double, 1e-9, VTK_DOUBLE_MAX);
  vtkGetMacro(Radius, double);

  vtkSetClampMacro(Dx, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(Dx, double);
  vtkSetClampMacro(Dy, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(Dy, double);
  vtkSetClampMacro(Dz, double, 1e-6, VTK_DOUBLE_MAX);
  vtkGetMacro(Dz, double);

  vtkSetClampMacro(MinimumNumberOfSides, int, 8, 512);
  vtkGetMacro(MinimumNumberOfSides, int);

  virtual void SetTopSurface(vtkPolyData* surface);
  vtkGetObjectMacro(TopSurface, vtkPolyData);
  virtual void SetBottomSurface(vtkPolyData* surface);
  vtkGetObjectMacro(BottomSurface, vtkPolyData);

protected:
  vtkBoreholeSurfaceFilter();
  ~vtkBoreholeSurfaceFilter() override;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;

  bool EvaluateSurfaceHeight(vtkPolyData* surface, double x, double y, double& z) const;

  double Radius;
  double Dx;
  double Dy;
  double Dz;
  int MinimumNumberOfSides;
  vtkPolyData* TopSurface;
  vtkPolyData* BottomSurface;

private:
  vtkBoreholeSurfaceFilter(const vtkBoreholeSurfaceFilter&) = delete;
  void operator=(const vtkBoreholeSurfaceFilter&) = delete;
};

VTK_ABI_NAMESPACE_END

#endif
