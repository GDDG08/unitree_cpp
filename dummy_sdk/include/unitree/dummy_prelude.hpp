// Dummy unitree_sdk2 — shared prelude.
//
// This is a header-only, no-op stand-in for the real Unitree SDK2 + CycloneDDS,
// used to build/import `unitree_cpp` on platforms where the real SDK cannot be
// installed (e.g. macOS). It provides exactly the symbols `unitree_controller`
// references — nothing connects to hardware, publishers drop writes and
// subscribers never fire, so `self_check()` will report no data at runtime.
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <sys/types.h>  // uint
#include <unistd.h>     // sleep

// Generate a value-semantic field with const/non-const accessors, matching the
// IDL message style `obj.field() = x` / `x = obj.field()` used by the SDK.
// The type is variadic so that template types with commas (e.g. std::array<T, N>)
// pass through as a single argument.
#define UT_DUMMY_FIELD(name, ...)                       \
    __VA_ARGS__ _##name{};                              \
    __VA_ARGS__& name() { return _##name; }             \
    const __VA_ARGS__& name() const { return _##name; }

namespace unitree {
namespace common {

#ifndef UT_CPU_ID_NONE
#define UT_CPU_ID_NONE (-1)
#endif

// Opaque recurrent-thread handle. The dummy never actually runs the callback.
class Thread {
   public:
    virtual ~Thread() = default;
};
using ThreadPtr = std::shared_ptr<Thread>;

template <typename... Args>
inline ThreadPtr CreateRecurrentThreadEx(Args&&...) {
    return std::make_shared<Thread>();
}

}  // namespace common
}  // namespace unitree
