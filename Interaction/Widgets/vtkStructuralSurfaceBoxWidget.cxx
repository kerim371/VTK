// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkStructuralSurfaceBoxWidget.h"

#include "vtkCallbackCommand.h"
#include "vtkCommand.h"
#include "vtkObjectFactory.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkStructuralSurfaceBoxRepresentation.h"
#include "vtkWidgetCallbackMapper.h"
#include "vtkWidgetEvent.h"

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkStructuralSurfaceBoxWidget);

vtkStructuralSurfaceBoxWidget::vtkStructuralSurfaceBoxWidget()
{
  this->WidgetState = Start;

  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonPressEvent, vtkWidgetEvent::Select,
    this, vtkStructuralSurfaceBoxWidget::SelectAction);
  this->CallbackMapper->SetCallbackMethod(
    vtkCommand::MouseMoveEvent, vtkWidgetEvent::Move, this, vtkStructuralSurfaceBoxWidget::MoveAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent,
    vtkWidgetEvent::EndSelect, this, vtkStructuralSurfaceBoxWidget::EndSelectAction);
}

void vtkStructuralSurfaceBoxWidget::CreateDefaultRepresentation()
{
  if (!this->WidgetRep)
  {
    vtkStructuralSurfaceBoxRepresentation* rep = vtkStructuralSurfaceBoxRepresentation::New();
    this->WidgetRep = rep;
    rep->Delete();
  }
}

void vtkStructuralSurfaceBoxWidget::SetRepresentation(vtkStructuralSurfaceBoxRepresentation* rep)
{
  this->Superclass::SetWidgetRepresentation(rep);
}

vtkStructuralSurfaceBoxRepresentation*
vtkStructuralSurfaceBoxWidget::GetStructuralSurfaceBoxRepresentation()
{
  return reinterpret_cast<vtkStructuralSurfaceBoxRepresentation*>(this->WidgetRep);
}

void vtkStructuralSurfaceBoxWidget::SelectAction(vtkAbstractWidget* w)
{
  vtkStructuralSurfaceBoxWidget* self = reinterpret_cast<vtkStructuralSurfaceBoxWidget*>(w);
  vtkStructuralSurfaceBoxRepresentation* rep = self->GetStructuralSurfaceBoxRepresentation();
  if (!rep)
  {
    return;
  }

  const int* eventPos = self->Interactor->GetEventPosition();
  const int state = rep->ComputeInteractionState(eventPos[0], eventPos[1]);
  if (state == vtkStructuralSurfaceBoxRepresentation::Outside)
  {
    return;
  }

  double e[2] = { static_cast<double>(eventPos[0]), static_cast<double>(eventPos[1]) };
  rep->StartWidgetInteraction(e);
  self->WidgetState = Active;
  self->GrabFocus(self->EventCallbackCommand);
  self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

void vtkStructuralSurfaceBoxWidget::MoveAction(vtkAbstractWidget* w)
{
  vtkStructuralSurfaceBoxWidget* self = reinterpret_cast<vtkStructuralSurfaceBoxWidget*>(w);
  vtkStructuralSurfaceBoxRepresentation* rep = self->GetStructuralSurfaceBoxRepresentation();
  if (!rep)
  {
    return;
  }

  const int* eventPos = self->Interactor->GetEventPosition();
  if (self->WidgetState == Start)
  {
    rep->ComputeInteractionState(eventPos[0], eventPos[1]);
    self->Render();
    return;
  }

  double e[2] = { static_cast<double>(eventPos[0]), static_cast<double>(eventPos[1]) };
  rep->WidgetInteraction(e);
  self->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

void vtkStructuralSurfaceBoxWidget::EndSelectAction(vtkAbstractWidget* w)
{
  vtkStructuralSurfaceBoxWidget* self = reinterpret_cast<vtkStructuralSurfaceBoxWidget*>(w);
  if (self->WidgetState != Active)
  {
    return;
  }

  vtkStructuralSurfaceBoxRepresentation* rep = self->GetStructuralSurfaceBoxRepresentation();
  if (rep)
  {
    const int* eventPos = self->Interactor->GetEventPosition();
    double e[2] = { static_cast<double>(eventPos[0]), static_cast<double>(eventPos[1]) };
    rep->EndWidgetInteraction(e);
    rep->ComputeInteractionState(eventPos[0], eventPos[1]);
  }

  self->WidgetState = Start;
  self->ReleaseFocus();
  self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

void vtkStructuralSurfaceBoxWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "WidgetState: " << this->WidgetState << "\n";
}
VTK_ABI_NAMESPACE_END
