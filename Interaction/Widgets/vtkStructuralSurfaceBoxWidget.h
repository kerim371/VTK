// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkStructuralSurfaceBoxWidget
 * @brief   interactive widget for a structural-surface-constrained ROI
 */

#ifndef vtkStructuralSurfaceBoxWidget_h
#define vtkStructuralSurfaceBoxWidget_h

#include "vtkAbstractWidget.h"
#include "vtkCommand.h"
#include "vtkInteractionWidgetsModule.h" // For export macro
#include "vtkWrappingHints.h" // For VTK_MARSHALAUTO

VTK_ABI_NAMESPACE_BEGIN
class vtkStructuralSurfaceBoxRepresentation;

class VTKINTERACTIONWIDGETS_EXPORT VTK_MARSHALAUTO vtkStructuralSurfaceBoxWidget
  : public vtkAbstractWidget
{
public:
  static vtkStructuralSurfaceBoxWidget* New();
  vtkTypeMacro(vtkStructuralSurfaceBoxWidget, vtkAbstractWidget);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  // These custom events separate translation from resizing so applications can
  // react differently to each kind of user interaction.
  enum WidgetEventIds
  {
    TranslateStartEvent = vtkCommand::UserEvent + 100,
    TranslateInteractionEvent,
    TranslateEndEvent,
    ResizeStartEvent,
    ResizeInteractionEvent,
    ResizeEndEvent
  };

  void SetRepresentation(vtkStructuralSurfaceBoxRepresentation* rep);
  vtkStructuralSurfaceBoxRepresentation* GetStructuralSurfaceBoxRepresentation();

  void CreateDefaultRepresentation() override;

protected:
  vtkStructuralSurfaceBoxWidget();
  ~vtkStructuralSurfaceBoxWidget() override = default;

  enum WidgetStateType
  {
    Start = 0,
    Active
  };

  enum InteractionModeType
  {
    NoInteraction = 0,
    TranslationInteraction,
    ResizeInteraction
  };

  int WidgetState;
  int InteractionMode;

  static void SelectAction(vtkAbstractWidget* w);
  static void MoveAction(vtkAbstractWidget* w);
  static void EndSelectAction(vtkAbstractWidget* w);

private:
  vtkStructuralSurfaceBoxWidget(const vtkStructuralSurfaceBoxWidget&) = delete;
  void operator=(const vtkStructuralSurfaceBoxWidget&) = delete;
};

VTK_ABI_NAMESPACE_END
#endif
