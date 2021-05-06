#include "rigidbody.h"
#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include <algorithm>
#include <cmath>


Rigidbody::Rigidbody(Scene_Object& obj) : body(obj) {
    // TODO: Initialize pose here, or perhaps just modify it in update
}

Scene_Object& Rigidbody::obj() {
    return body;
}
