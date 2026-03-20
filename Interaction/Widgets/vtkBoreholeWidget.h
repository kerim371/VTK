// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkBoreholeWidget
 * @brief   direct-manipulation widget for borehole interval controls
 *
 * The widget supports hover highlighting and left-drag interaction:
 * - dragging top/bottom caps moves clipping positions along trajectory
 * - dragging wall changes tube radius
 */

#ifndef vtkBoreholeWidget_h
#define vtkBoreholeWidget_h

#include "vtkAbstractWidget.h"
#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWrappingHints.h" // For VTK_MARSHALAUTO

VTK_ABI_NAMESPACE_BEGIN
class vtkBoreholeRepresentation;

class VTKINTERACTIONWIDGETS_EXPORT VTK_MARSHALAUTO vtkBoreholeWidget : public vtkAbstractWidget
{
public:
  static vtkBoreholeWidget* New();
  vtkTypeMacro(vtkBoreholeWidget, vtkAbstractWidget);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  void SetRepresentation(vtkBoreholeRepresentation* rep);
  vtkBoreholeRepresentation* GetBoreholeRepresentation();

  void CreateDefaultRepresentation() override;

protected:
  vtkBoreholeWidget();
  ~vtkBoreholeWidget() override = default;

  enum WidgetStateType
  {
    Start = 0,
    Active
  };

  int WidgetState;

  static void SelectAction(vtkAbstractWidget* w);
  static void MoveAction(vtkAbstractWidget* w);
  static void EndSelectAction(vtkAbstractWidget* w);

private:
  vtkBoreholeWidget(const vtkBoreholeWidget&) = delete;
  void operator=(const vtkBoreholeWidget&) = delete;
};

VTK_ABI_NAMESPACE_END

#endif
