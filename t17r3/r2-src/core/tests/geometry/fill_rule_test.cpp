// Both headers that take a FillRule must be usable from one translation unit and agree on it.
#include "carvetto/geometry/fill_rule.hpp"

#include <catch2/catch_test_macros.hpp>
#include <initializer_list>
#include <type_traits>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"

using namespace carvetto::geometry;

TEST_CASE("FillRule is one type shared by booleans and point-in-contour", "[geometry][fill_rule]") {
    using BoolFn = BooleanResult (*)(BooleanOp, const std::vector<Contour>&, const std::vector<Contour>&,
                                     const Tolerance&, FillRule);
    using InsideFn = bool (*)(const Contour&, Vec2, FillRule, const Tolerance&);
    STATIC_CHECK(std::is_same_v<decltype(static_cast<BoolFn>(&booleanOp)), BoolFn>);
    STATIC_CHECK(std::is_same_v<decltype(static_cast<InsideFn>(&isInside)), InsideFn>);

    // A square traced twice (winding 2): filled under NonZero, empty under EvenOdd, for both APIs.
    const Tolerance tol{};
    Contour twice = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
    const Contour once = twice;
    twice.spans.insert(twice.spans.end(), once.spans.begin(), once.spans.end());
    for (const FillRule rule : {FillRule::NonZero, FillRule::EvenOdd}) {
        const bool filled = rule == FillRule::NonZero;
        CHECK(isInside(twice, {5.0, 5.0}, rule, tol) == filled);
        const BooleanResult u = booleanOp(BooleanOp::Union, {twice}, {}, tol, rule);
        CHECK(u.closed.empty() == !filled);
    }
}
