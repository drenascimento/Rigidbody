#include "rigidbody.h"
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include <algorithm>
#include <cmath>

Rigidbody::Rigidbody(GL::Mesh&& mesh) : body(std::move(mesh)) {
    // TODO: Initialize pose here, or perhaps just modify it in update
}

GL::Mesh& Rigidbody::mesh() {
    return body;
}
