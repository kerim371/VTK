// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
#include "vtkBoreholeWidget.h"

#include "vtkBoreholeRepresentation.h"
#include "vtkCallbackCommand.h"
#include "vtkCommand.h"
#include "vtkObjectFactory.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkWidgetCallbackMapper.h"
#include "vtkWidgetEvent.h"

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkBoreholeWidget);

vtkBoreholeWidget::vtkBoreholeWidget()
{
  this->WidgetState = Start;

  this->CallbackMapper->SetCallbackMethod(
    vtkCommand::LeftButtonPressEvent, vtkWidgetEvent::Select, this, vtkBoreholeWidget::SelectAction);
  this->CallbackMapper->SetCallbackMethod(
    vtkCommand::MouseMoveEvent, vtkWidgetEvent::Move, this, vtkBoreholeWidget::MoveAction);
  this->CallbackMapper->SetCallbackMethod(vtkCommand::LeftButtonReleaseEvent, vtkWidgetEvent::EndSelect,
    this, vtkBoreholeWidget::EndSelectAction);
}

void vtkBoreholeWidget::CreateDefaultRepresentation()
{
  if (!this->WidgetRep)
  {
    vtkBoreholeRepresentation* rep = vtkBoreholeRepresentation::New();
    this->WidgetRep = rep;
    rep->Delete();
  }
}

void vtkBoreholeWidget::SetRepresentation(vtkBoreholeRepresentation* rep)
{
  this->Superclass::SetWidgetRepresentation(rep);
}

vtkBoreholeRepresentation* vtkBoreholeWidget::GetBoreholeRepresentation()
{
  return reinterpret_cast<vtkBoreholeRepresentation*>(this->WidgetRep);
}

void vtkBoreholeWidget::SelectAction(vtkAbstractWidget* w)
{
  vtkBoreholeWidget* self = reinterpret_cast<vtkBoreholeWidget*>(w);
  vtkBoreholeRepresentation* rep = self->GetBoreholeRepresentation();
  if (!rep)
  {
    return;
  }

  const int* eventPos = self->Interactor->GetEventPosition();
  const int state = rep->ComputeInteractionState(eventPos[0], eventPos[1]);

  int op = vtkBoreholeRepresentation::DragNone;
  if (state == vtkBoreholeRepresentation::OverTopGlyph)
  {
    op = vtkBoreholeRepresentation::DragTopCap;
  }
  else if (state == vtkBoreholeRepresentation::OverBottomGlyph)
  {
    op = vtkBoreholeRepresentation::DragBottomCap;
  }
  else if (state == vtkBoreholeRepresentation::OverWall)
  {
    op = vtkBoreholeRepresentation::DragWallRadius;
  }

  if (op == vtkBoreholeRepresentation::DragNone)
  {
    return;
  }

  rep->SetCurrentOperation(op);
  double e[2] = { static_cast<double>(eventPos[0]), static_cast<double>(eventPos[1]) };
  rep->StartWidgetInteraction(e);

  self->WidgetState = Active;
  self->GrabFocus(self->EventCallbackCommand);
  self->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

void vtkBoreholeWidget::MoveAction(vtkAbstractWidget* w)
{
  vtkBoreholeWidget* self = reinterpret_cast<vtkBoreholeWidget*>(w);
  vtkBoreholeRepresentation* rep = self->GetBoreholeRepresentation();
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

void vtkBoreholeWidget::EndSelectAction(vtkAbstractWidget* w)
{
  vtkBoreholeWidget* self = reinterpret_cast<vtkBoreholeWidget*>(w);
  if (self->WidgetState != Active)
  {
    return;
  }

  vtkBoreholeRepresentation* rep = self->GetBoreholeRepresentation();
  if (rep)
  {
    const int* eventPos = self->Interactor->GetEventPosition();
    double e[2] = { static_cast<double>(eventPos[0]), static_cast<double>(eventPos[1]) };
    rep->EndWidgetInteraction(e);
    rep->SetCurrentOperation(vtkBoreholeRepresentation::DragNone);
  }

  self->WidgetState = Start;
  self->ReleaseFocus();
  self->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
  self->EventCallbackCommand->SetAbortFlag(1);
  self->Render();
}

void vtkBoreholeWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "WidgetState: " << this->WidgetState << "\n";
}
VTK_ABI_NAMESPACE_END
