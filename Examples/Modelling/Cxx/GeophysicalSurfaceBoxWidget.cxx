#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <vector>

#include <vtkActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkDataSetMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderRepresentation.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

namespace
{
class SurfaceStack
{
public:
  int Nx = 0;
  int Ny = 0;
  double X0 = 0.0;
  double Y0 = 0.0;
  double Dx = 1.0;
  double Dy = 1.0;
  std::vector<std::vector<double>> Layers; // [layer][j * Nx + i]

  double SampleLayer(int layer, double x, double y) const
  {
    layer = std::clamp(layer, 0, static_cast<int>(this->Layers.size()) - 1);
    const double gx = (x - this->X0) / this->Dx;
    const double gy = (y - this->Y0) / this->Dy;

    const int i0 = std::clamp(static_cast<int>(std::floor(gx)), 0, this->Nx - 2);
    const int j0 = std::clamp(static_cast<int>(std::floor(gy)), 0, this->Ny - 2);
    const int i1 = i0 + 1;
    const int j1 = j0 + 1;

    const double tx = std::clamp(gx - i0, 0.0, 1.0);
    const double ty = std::clamp(gy - j0, 0.0, 1.0);

    auto val = [&](int i, int j) { return this->Layers[layer][j * this->Nx + i]; };

    const double z00 = val(i0, j0);
    const double z10 = val(i1, j0);
    const double z01 = val(i0, j1);
    const double z11 = val(i1, j1);

    const double z0 = z00 * (1.0 - tx) + z10 * tx;
    const double z1 = z01 * (1.0 - tx) + z11 * tx;
    return z0 * (1.0 - ty) + z1 * ty;
  }

  double SampleInterpolated(double layerPosition, double x, double y) const
  {
    const int layerCount = static_cast<int>(this->Layers.size());
    const double p = std::clamp(layerPosition, 0.0, static_cast<double>(layerCount - 1));
    const int k0 = static_cast<int>(std::floor(p));
    const int k1 = std::min(k0 + 1, layerCount - 1);
    const double tk = p - k0;

    const double z0 = this->SampleLayer(k0, x, y);
    const double z1 = this->SampleLayer(k1, x, y);
    return z0 * (1.0 - tk) + z1 * tk;
  }

  static SurfaceStack MakeSynthetic(int nx, int ny, int numLayers)
  {
    SurfaceStack out;
    out.Nx = nx;
    out.Ny = ny;
    out.X0 = 0.0;
    out.Y0 = 0.0;
    out.Dx = 1.0;
    out.Dy = 1.0;

    out.Layers.resize(numLayers);
    for (int k = 0; k < numLayers; ++k)
    {
      auto& layer = out.Layers[k];
      layer.resize(nx * ny);
      const double baseDepth = 40.0 + 22.0 * k;
      const double waviness = 8.0 + 1.5 * k;

      for (int j = 0; j < ny; ++j)
      {
        for (int i = 0; i < nx; ++i)
        {
          const double x = static_cast<double>(i) / (nx - 1);
          const double y = static_cast<double>(j) / (ny - 1);
          const double z = baseDepth + waviness * std::sin(6.0 * x + 1.1 * k) * std::cos(5.0 * y - 0.7 * k) +
            6.0 * std::exp(-11.0 * ((x - 0.55) * (x - 0.55) + (y - 0.45) * (y - 0.45)));
          layer[j * nx + i] = z;
        }
      }
    }
    return out;
  }
};

class InterpolatedTopVolume
{
public:
  void SetSurfaceStack(SurfaceStack stack)
  {
    this->Stack = std::move(stack);
  }

  vtkUnstructuredGrid* GetOutput()
  {
    return this->Grid;
  }

  void Update(double xmin, double xmax, double ymin, double ymax, double layerPos, double topShift,
    double thickness)
  {
    const int sx = this->ResolutionX;
    const int sy = this->ResolutionY;

    vtkNew<vtkPoints> pts;
    pts->SetNumberOfPoints(static_cast<vtkIdType>(2 * sx * sy));

    auto pointId = [&](int i, int j, int side) {
      return static_cast<vtkIdType>(side * sx * sy + j * sx + i);
    };

    for (int j = 0; j < sy; ++j)
    {
      const double v = static_cast<double>(j) / (sy - 1);
      const double y = ymin + (ymax - ymin) * v;
      for (int i = 0; i < sx; ++i)
      {
        const double u = static_cast<double>(i) / (sx - 1);
        const double x = xmin + (xmax - xmin) * u;
        const double zTop = this->Stack.SampleInterpolated(layerPos, x, y) + topShift;
        const double zBot = zTop - thickness;

        pts->SetPoint(pointId(i, j, 1), x, y, zTop);
        pts->SetPoint(pointId(i, j, 0), x, y, zBot);
      }
    }

    vtkNew<vtkCellArray> cells;
    for (int j = 0; j < sy - 1; ++j)
    {
      for (int i = 0; i < sx - 1; ++i)
      {
        std::array<vtkIdType, 8> hex = {
          pointId(i, j, 0), pointId(i + 1, j, 0), pointId(i + 1, j + 1, 0), pointId(i, j + 1, 0),
          pointId(i, j, 1), pointId(i + 1, j, 1), pointId(i + 1, j + 1, 1), pointId(i, j + 1, 1)
        };
        cells->InsertNextCell(8, hex.data());
      }
    }

    this->Grid->SetPoints(pts);
    this->Grid->SetCells(VTK_HEXAHEDRON, cells);
    this->Grid->Modified();
  }

  int ResolutionX = 24;
  int ResolutionY = 24;

private:
  SurfaceStack Stack;
  vtkNew<vtkUnstructuredGrid> Grid;
};

struct AppState
{
  SurfaceStack Stack;
  InterpolatedTopVolume Volume;
  vtkSmartPointer<vtkBoxRepresentation> BoxRep;
  double LayerPosition = 0.0;
  double TopShift = 0.0;
  double Thickness = 25.0;
};

void RebuildVolume(AppState* state)
{
  double b[6];
  state->BoxRep->GetBounds(b);
  const double cx = 0.5 * (b[0] + b[1]);
  const double cy = 0.5 * (b[2] + b[3]);

  const double zCenterSurface = state->Stack.SampleInterpolated(state->LayerPosition, cx, cy);
  state->TopShift = b[5] - zCenterSurface;
  state->Thickness = std::max(3.0, b[5] - b[4]);

  state->Volume.Update(b[0], b[1], b[2], b[3], state->LayerPosition, state->TopShift, state->Thickness);
}

void OnBoxInteraction(vtkObject* caller, unsigned long, void* clientData, void*)
{
  auto* state = static_cast<AppState*>(clientData);
  auto* widget = reinterpret_cast<vtkBoxWidget2*>(caller);
  auto* rep = reinterpret_cast<vtkBoxRepresentation*>(widget->GetRepresentation());

  state->BoxRep = rep;
  RebuildVolume(state);
}

void OnSliderInteraction(vtkObject* caller, unsigned long, void* clientData, void*)
{
  auto* state = static_cast<AppState*>(clientData);
  auto* sliderWidget = reinterpret_cast<vtkSliderWidget*>(caller);
  auto* rep = reinterpret_cast<vtkSliderRepresentation*>(sliderWidget->GetRepresentation());
  state->LayerPosition = rep->GetValue();

  double b[6];
  state->BoxRep->GetBounds(b);
  const double cx = 0.5 * (b[0] + b[1]);
  const double cy = 0.5 * (b[2] + b[3]);
  const double zTop = state->Stack.SampleInterpolated(state->LayerPosition, cx, cy) + state->TopShift;
  b[4] = zTop - state->Thickness;
  b[5] = zTop;
  state->BoxRep->PlaceWidget(b);

  RebuildVolume(state);
}
}

int main(int, char*[])
{
  vtkNew<vtkNamedColors> colors;

  AppState state;
  state.Stack = SurfaceStack::MakeSynthetic(96, 96, 5);
  state.Volume.SetSurfaceStack(state.Stack);

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(1280, 820);
  renderWindow->SetWindowName("GeophysicalInterpolatedTopWidget");

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  interactor->SetInteractorStyle(style);

  vtkNew<vtkDataSetMapper> volumeMapper;
  volumeMapper->SetInputData(state.Volume.GetOutput());

  vtkNew<vtkActor> volumeActor;
  volumeActor->SetMapper(volumeMapper);
  volumeActor->GetProperty()->SetColor(colors->GetColor3d("Wheat").GetData());
  volumeActor->GetProperty()->SetOpacity(0.72);
  volumeActor->GetProperty()->SetEdgeVisibility(true);
  volumeActor->GetProperty()->SetEdgeColor(colors->GetColor3d("Black").GetData());
  renderer->AddActor(volumeActor);

  vtkNew<vtkBoxWidget2> boxWidget;
  boxWidget->SetInteractor(interactor);

  vtkNew<vtkBoxRepresentation> boxRep;
  const double initBounds[6] = { 20.0, 75.0, 18.0, 78.0, 20.0, 55.0 };
  boxRep->PlaceWidget(initBounds);
  boxWidget->SetRepresentation(boxRep);
  boxWidget->GetRepresentation()->SetPlaceFactor(1.0);

  state.BoxRep = boxRep;
  state.LayerPosition = 1.6;
  RebuildVolume(&state);

  vtkNew<vtkCallbackCommand> boxCb;
  boxCb->SetCallback(OnBoxInteraction);
  boxCb->SetClientData(&state);
  boxWidget->AddObserver(vtkCommand::InteractionEvent, boxCb);

  vtkNew<vtkSliderRepresentation2D> sliderRep;
  sliderRep->SetMinimumValue(0.0);
  sliderRep->SetMaximumValue(static_cast<double>(state.Stack.Layers.size() - 1));
  sliderRep->SetValue(state.LayerPosition);
  sliderRep->SetTitleText("Interpolated layer index");
  sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint1Coordinate()->SetValue(0.08, 0.1);
  sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint2Coordinate()->SetValue(0.42, 0.1);
  sliderRep->SetSliderLength(0.025);
  sliderRep->SetSliderWidth(0.03);
  sliderRep->SetTubeWidth(0.008);
  sliderRep->SetLabelFormat("%.2f");

  vtkNew<vtkSliderWidget> sliderWidget;
  sliderWidget->SetInteractor(interactor);
  sliderWidget->SetRepresentation(sliderRep);
  sliderWidget->SetAnimationModeToAnimate();
  sliderWidget->EnabledOn();

  vtkNew<vtkCallbackCommand> sliderCb;
  sliderCb->SetCallback(OnSliderInteraction);
  sliderCb->SetClientData(&state);
  sliderWidget->AddObserver(vtkCommand::InteractionEvent, sliderCb);

  boxWidget->On();

  renderer->ResetCamera();
  renderer->GetActiveCamera()->Azimuth(35.0);
  renderer->GetActiveCamera()->Elevation(25.0);
  renderer->ResetCameraClippingRange();

  renderWindow->Render();
  interactor->Initialize();
  interactor->Start();

  return EXIT_SUCCESS;
}
