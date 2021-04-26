#pragma once

#include "rigidbody.h"
#include "../lib/mathlib.h"

#include <algorithm>
#include <cmath>

Scene_Rigidbody::Scene_Rigidbody(Scene_ID id) {
    _id = id;
    // TODO: Initialize Mesh
    // TODO: Initialize Particles
    snprintf(opt.name, max_name_len, "Emitter %d", id);
}

// TODO: Implement rest of the methods.
