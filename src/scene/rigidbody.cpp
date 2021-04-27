#pragma once

#include "rigidbody.h"
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include <algorithm>
#include <cmath>

Rigidbody::Rigidbody(GL::Mesh mesh) {
    _mesh = mesh;
}



GL::Mesh Rigidbody::mesh() {
    return _mesh;
}
