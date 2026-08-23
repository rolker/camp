// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "shoreline_anchor.h"

#include <array>

namespace camp
{
namespace raster
{

namespace
{

// D3 precedence, high to low. Kept in one place so mode() resolution and the
// declaration order in the header cannot drift apart.
constexpr std::array<ShorelineAnchor::Source, 4> kPrecedence = {
  ShorelineAnchor::Source::ChartDatum,
  ShorelineAnchor::Source::PlatformTide,
  ShorelineAnchor::Source::Manual,
  ShorelineAnchor::Source::None,
};

std::size_t precedenceIndex(ShorelineAnchor::Source source)
{
  for(std::size_t i = 0; i < kPrecedence.size(); ++i)
    if(kPrecedence[i] == source)
      return i;
  return kPrecedence.size() - 1;  // unreachable; keeps the walk bounded
}

}  // namespace

ShorelineAnchor::ShorelineAnchor(QObject * parent) : QObject(parent) {}

std::optional<double> ShorelineAnchor::valueFor(Source source) const
{
  switch(source)
  {
    case Source::ChartDatum:   return chart_datum_;
    case Source::PlatformTide: return platform_tide_;
    case Source::Manual:       return manual_;
    case Source::None:         return std::nullopt;
  }
  return std::nullopt;
}

std::optional<double> ShorelineAnchor::value() const
{
  // Walk downward from the active mode to the first source with a value. This is
  // the D3 "fallback order applied beneath whichever mode is active": a
  // ChartDatum mode with no datum still resolves through platform tide and manual;
  // a Manual mode never reaches back up to chart datum; None resolves to nothing.
  for(std::size_t i = precedenceIndex(mode_); i < kPrecedence.size(); ++i)
    if(const std::optional<double> v = valueFor(kPrecedence[i]))
      return v;
  return std::nullopt;
}

ShorelineAnchor::Source ShorelineAnchor::activeSource() const
{
  for(std::size_t i = precedenceIndex(mode_); i < kPrecedence.size(); ++i)
    if(valueFor(kPrecedence[i]))
      return kPrecedence[i];
  return Source::None;
}

void ShorelineAnchor::setMode(Source mode)
{
  if(mode == mode_)
    return;
  const std::optional<double> before = value();
  const Source before_source = activeSource();
  mode_ = mode;
  if(value() != before || activeSource() != before_source)
    emit changed();
}

void ShorelineAnchor::setChartDatum(std::optional<double> value)
{
  if(value == chart_datum_)
    return;
  const std::optional<double> before = this->value();
  const Source before_source = activeSource();
  chart_datum_ = value;
  if(this->value() != before || activeSource() != before_source)
    emit changed();
}

void ShorelineAnchor::setPlatformTide(std::optional<double> value)
{
  if(value == platform_tide_)
    return;
  const std::optional<double> before = this->value();
  const Source before_source = activeSource();
  platform_tide_ = value;
  if(this->value() != before || activeSource() != before_source)
    emit changed();
}

void ShorelineAnchor::setManual(std::optional<double> value)
{
  if(value == manual_)
    return;
  const std::optional<double> before = this->value();
  const Source before_source = activeSource();
  manual_ = value;
  if(this->value() != before || activeSource() != before_source)
    emit changed();
}

QString ShorelineAnchor::sourceLabel(Source source)
{
  switch(source)
  {
    case Source::ChartDatum:   return QStringLiteral("chart datum");
    case Source::PlatformTide: return QStringLiteral("platform tide");
    case Source::Manual:       return QStringLiteral("manual");
    case Source::None:         return QStringLiteral("none");
  }
  return QStringLiteral("none");
}

}  // namespace raster
}  // namespace camp
