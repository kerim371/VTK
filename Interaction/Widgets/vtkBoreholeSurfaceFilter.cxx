// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

#include "vtkBoreholeSurfaceFilter.h"

#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkDataObject.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"

#include <algorithm>
#include <cmath>
#include <vector>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(vtkBoreholeSurfaceFilter);

vtkBoreholeSurfaceFilter::vtkBoreholeSurfaceFilter()
  : Radius(1.0)
  , Dx(1.0)
  , Dy(1.0)
  , Dz(1.0)
  , MinimumNumberOfSides(24)
  , TopSurface(nullptr)
  , BottomSurface(nullptr)
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

vtkBoreholeSurfaceFilter::~vtkBoreholeSurfaceFilter()
{
  this->SetTopSurface(nullptr);
  this->SetBottomSurface(nullptr);
}

void vtkBoreholeSurfaceFilter::SetTopSurface(vtkPolyData* surface)
{
  if (this->TopSurface == surface)
  {
    return;
  }
  if (this->TopSurface)
  {
    this->TopSurface->UnRegister(this);
  }
  this->TopSurface = surface;
  if (this->TopSurface)
  {
    this->TopSurface->Register(this);
  }
  this->Modified();
}

void vtkBoreholeSurfaceFilter::SetBottomSurface(vtkPolyData* surface)
{
  if (this->BottomSurface == surface)
  {
    return;
  }
  if (this->BottomSurface)
  {
    this->BottomSurface->UnRegister(this);
  }
  this->BottomSurface = surface;
  if (this->BottomSurface)
  {
    this->BottomSurface->Register(this);
  }
  this->Modified();
}

bool vtkBoreholeSurfaceFilter::EvaluateSurfaceHeight(
  vtkPolyData* surface, double x, double y, double& z) const
{
  if (!surface || !surface->GetPoints())
  {
    return false;
  }

  double bounds[6];
  surface->GetBounds(bounds);
  if (x < bounds[0] || x > bounds[1] || y < bounds[2] || y > bounds[3])
  {
    return false;
  }

  vtkPoints* pts = surface->GetPoints();
  vtkIdType bestId = -1;
  double bestD2 = VTK_DOUBLE_MAX;
  for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i)
  {
    double p[3];
    pts->GetPoint(i, p);
    const double dx = p[0] - x;
    const double dy = p[1] - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < bestD2)
    {
      bestD2 = d2;
      bestId = i;
    }
  }

  if (bestId < 0)
  {
    return false;
  }

  double p[3];
  pts->GetPoint(bestId, p);
  z = p[2];
  return true;
}

int vtkBoreholeSurfaceFilter::RequestData(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkPolyData* input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkNew<vtkPoints> outPts;
  vtkNew<vtkCellArray> outPolys;
  output->SetPoints(outPts);
  output->SetPolys(outPolys);

  if (!input || !input->GetPoints() || input->GetNumberOfPoints() < 2)
  {
    return 1;
  }

  vtkPoints* inPts = input->GetPoints();
  const vtkIdType nIn = inPts->GetNumberOfPoints();

  std::vector<double> cumulative(static_cast<size_t>(nIn), 0.0);
  double totalLength = 0.0;
  for (vtkIdType i = 1; i < nIn; ++i)
  {
    double p0[3], p1[3];
    inPts->GetPoint(i - 1, p0);
    inPts->GetPoint(i, p1);
    totalLength += std::sqrt(vtkMath::Distance2BetweenPoints(p0, p1));
    cumulative[static_cast<size_t>(i)] = totalLength;
  }
  if (totalLength <= 0.0)
  {
    return 1;
  }

  const int axialSegments = std::max(1, static_cast<int>(std::ceil(totalLength / this->Dz)));
  const double meanStep = 0.5 * (this->Dx + this->Dy);
  const int ringSize = std::max(this->MinimumNumberOfSides,
    static_cast<int>(std::ceil((2.0 * vtkMath::Pi() * this->Radius) / meanStep)));

  auto sampleAtS = [&](double s, double p[3], double t[3]) {
    s = std::clamp(s, 0.0, totalLength);
    vtkIdType seg = 0;
    for (vtkIdType i = 1; i < nIn; ++i)
    {
      if (cumulative[static_cast<size_t>(i)] >= s)
      {
        seg = i - 1;
        break;
      }
      seg = i - 1;
    }

    double p0[3], p1[3];
    inPts->GetPoint(seg, p0);
    inPts->GetPoint(seg + 1, p1);
    const double s0 = cumulative[static_cast<size_t>(seg)];
    const double s1 = cumulative[static_cast<size_t>(seg + 1)];
    const double segLen = std::max(s1 - s0, 1e-12);
    const double u = std::clamp((s - s0) / segLen, 0.0, 1.0);
    for (int k = 0; k < 3; ++k)
    {
      p[k] = p0[k] + u * (p1[k] - p0[k]);
      t[k] = p1[k] - p0[k];
    }
    vtkMath::Normalize(t);
  };

  std::vector<double> centers(static_cast<size_t>(axialSegments + 1) * 3);
  std::vector<double> tangents(static_cast<size_t>(axialSegments + 1) * 3);
  for (int i = 0; i <= axialSegments; ++i)
  {
    double p[3], t[3];
    sampleAtS((static_cast<double>(i) / axialSegments) * totalLength, p, t);
    for (int k = 0; k < 3; ++k)
    {
      centers[static_cast<size_t>(3 * i + k)] = p[k];
      tangents[static_cast<size_t>(3 * i + k)] = t[k];
    }
  }

  double ref[3] = { 0.0, 0.0, 1.0 };
  if (std::abs(vtkMath::Dot(ref, tangents.data())) > 0.95)
  {
    ref[0] = 0.0;
    ref[1] = 1.0;
    ref[2] = 0.0;
  }

  std::vector<double> ringPts(static_cast<size_t>(axialSegments + 1) * ringSize * 3, 0.0);
  double nPrev[3] = { 0.0, 0.0, 0.0 };
  vtkMath::Cross(tangents.data(), ref, nPrev);
  if (vtkMath::Normalize(nPrev) <= 0.0)
  {
    nPrev[0] = 1.0;
    nPrev[1] = 0.0;
    nPrev[2] = 0.0;
  }

  auto rotateAroundAxis = [](const double v[3], const double axis[3], double angle, double out[3]) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double dot = vtkMath::Dot(axis, v);
    double cross[3];
    vtkMath::Cross(axis, v, cross);
    for (int k = 0; k < 3; ++k)
    {
      out[k] = c * v[k] + s * cross[k] + (1.0 - c) * dot * axis[k];
    }
  };

  for (int i = 0; i <= axialSegments; ++i)
  {
    const double* c = &centers[static_cast<size_t>(3 * i)];
    const double* t = &tangents[static_cast<size_t>(3 * i)];

    double nCur[3] = { nPrev[0], nPrev[1], nPrev[2] };
    if (i > 0)
    {
      const double* tPrev = &tangents[static_cast<size_t>(3 * (i - 1))];
      double rotAxis[3];
      vtkMath::Cross(tPrev, t, rotAxis);
      const double sinTheta = vtkMath::Norm(rotAxis);
      const double cosTheta = std::clamp(vtkMath::Dot(tPrev, t), -1.0, 1.0);

      if (sinTheta > 1e-10)
      {
        for (int k = 0; k < 3; ++k)
        {
          rotAxis[k] /= sinTheta;
        }
        const double angle = std::atan2(sinTheta, cosTheta);
        double transported[3];
        rotateAroundAxis(nPrev, rotAxis, angle, transported);
        nCur[0] = transported[0];
        nCur[1] = transported[1];
        nCur[2] = transported[2];
      }
      else if (cosTheta < 0.0)
      {
        nCur[0] = -nCur[0];
        nCur[1] = -nCur[1];
        nCur[2] = -nCur[2];
      }
    }

    const double proj = vtkMath::Dot(nCur, t);
    for (int k = 0; k < 3; ++k)
    {
      nCur[k] -= proj * t[k];
    }
    if (vtkMath::Normalize(nCur) <= 0.0)
    {
      double fallback[3] = { 1.0, 0.0, 0.0 };
      if (std::abs(vtkMath::Dot(fallback, t)) > 0.9)
      {
        fallback[0] = 0.0;
        fallback[1] = 1.0;
      }
      vtkMath::Cross(t, fallback, nCur);
      vtkMath::Normalize(nCur);
    }
    double bCur[3];
    vtkMath::Cross(t, nCur, bCur);
    vtkMath::Normalize(bCur);

    for (int j = 0; j < ringSize; ++j)
    {
      const double ang = (2.0 * vtkMath::Pi() * j) / ringSize;
      const double cs = std::cos(ang);
      const double sn = std::sin(ang);
      double p[3];
      for (int k = 0; k < 3; ++k)
      {
        p[k] = c[k] + this->Radius * (cs * nCur[k] + sn * bCur[k]);
      }

      const size_t idx = static_cast<size_t>((i * ringSize + j) * 3);
      ringPts[idx + 0] = p[0];
      ringPts[idx + 1] = p[1];
      ringPts[idx + 2] = p[2];
    }

    nPrev[0] = nCur[0];
    nPrev[1] = nCur[1];
    nPrev[2] = nCur[2];
  }

  for (int i = 0; i <= axialSegments; ++i)
  {
    for (int j = 0; j < ringSize; ++j)
    {
      const size_t idx = static_cast<size_t>((i * ringSize + j) * 3);
      outPts->InsertNextPoint(&ringPts[idx]);
    }
  }

  for (int i = 0; i < axialSegments; ++i)
  {
    for (int j = 0; j < ringSize; ++j)
    {
      const vtkIdType a = static_cast<vtkIdType>(i * ringSize + j);
      const vtkIdType b = static_cast<vtkIdType>(i * ringSize + ((j + 1) % ringSize));
      const vtkIdType c = static_cast<vtkIdType>((i + 1) * ringSize + ((j + 1) % ringSize));
      const vtkIdType d = static_cast<vtkIdType>((i + 1) * ringSize + j);

      vtkIdType tri1[3] = { a, b, c };
      vtkIdType tri2[3] = { a, c, d };
      outPolys->InsertNextCell(3, tri1);
      outPolys->InsertNextCell(3, tri2);
    }
  }

  double topCenter[3] = { 0.0, 0.0, 0.0 };
  double bottomCenter[3] = { 0.0, 0.0, 0.0 };
  for (int j = 0; j < ringSize; ++j)
  {
    double pTop[3], pBottom[3];
    outPts->GetPoint(j, pTop);
    outPts->GetPoint(axialSegments * ringSize + j, pBottom);
    for (int k = 0; k < 3; ++k)
    {
      topCenter[k] += pTop[k];
      bottomCenter[k] += pBottom[k];
    }
  }
  for (int k = 0; k < 3; ++k)
  {
    topCenter[k] /= ringSize;
    bottomCenter[k] /= ringSize;
  }

  double zTop = 0.0;
  if (this->EvaluateSurfaceHeight(this->TopSurface, topCenter[0], topCenter[1], zTop))
  {
    topCenter[2] = zTop;
  }
  double zBottom = 0.0;
  if (this->EvaluateSurfaceHeight(this->BottomSurface, bottomCenter[0], bottomCenter[1], zBottom))
  {
    bottomCenter[2] = zBottom;
  }

  const vtkIdType topCenterId = outPts->InsertNextPoint(topCenter);
  const vtkIdType bottomCenterId = outPts->InsertNextPoint(bottomCenter);
  for (int j = 0; j < ringSize; ++j)
  {
    vtkIdType topTri[3] = { topCenterId, static_cast<vtkIdType>((j + 1) % ringSize), static_cast<vtkIdType>(j) };
    outPolys->InsertNextCell(3, topTri);

    const vtkIdType b0 = static_cast<vtkIdType>(axialSegments * ringSize + j);
    const vtkIdType b1 = static_cast<vtkIdType>(axialSegments * ringSize + ((j + 1) % ringSize));
    vtkIdType bottomTri[3] = { bottomCenterId, b0, b1 };
    outPolys->InsertNextCell(3, bottomTri);
  }

  output->Squeeze();
  return 1;
}

void vtkBoreholeSurfaceFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Radius: " << this->Radius << "\n";
  os << indent << "Dx: " << this->Dx << "\n";
  os << indent << "Dy: " << this->Dy << "\n";
  os << indent << "Dz: " << this->Dz << "\n";
  os << indent << "MinimumNumberOfSides: " << this->MinimumNumberOfSides << "\n";
  os << indent << "TopSurface: " << this->TopSurface << "\n";
  os << indent << "BottomSurface: " << this->BottomSurface << "\n";
}

VTK_ABI_NAMESPACE_END
