#pragma once

#include "eigen.hh"

#include "geometry/tri_mesh.hh"
#include "util/optional.hh"

#include <string>
#include <vector>

namespace geometry {
namespace import {

jcc::Optional<TriMesh> read_stl(const std::string &file_path);
}
}
