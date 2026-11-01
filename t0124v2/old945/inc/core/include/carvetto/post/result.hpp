#pragma once

#include <string>
#include <utility>
#include <variant>

namespace carvetto::post {

/// A recoverable failure, tied to the 1-based `.pp` source line it concerns (0 = no line).
struct Error {
    int line{0};
    std::string message;

    /// "line N: message", or just the message when there is no line.
    std::string describe() const {
        return line > 0 ? "line " + std::to_string(line) + ": " + message : message;
    }
};

/// A non-fatal note produced while parsing or running a post, with its source line (0 = none).
struct Diagnostic {
    int line{0};
    std::string message;

    /// "line N: message", or just the message when there is no line.
    std::string describe() const {
        return line > 0 ? "line " + std::to_string(line) + ": " + message : message;
    }
};

/// Value-or-error return type (a small stand-in for C++23 `std::expected`; ADR-0011).
template <class T>
class Result {
   public:
    Result(T value) : v_(std::move(value)) {}      // NOLINT(google-explicit-constructor)
    Result(Error error) : v_(std::move(error)) {}  // NOLINT(google-explicit-constructor)

    /// True when a value is held.
    bool ok() const { return v_.index() == 0; }
    explicit operator bool() const { return ok(); }

    /// The value; only valid when `ok()`.
    T& value() { return std::get<0>(v_); }
    /// The value; only valid when `ok()`.
    const T& value() const { return std::get<0>(v_); }
    /// The error; only valid when `!ok()`.
    const Error& error() const { return std::get<1>(v_); }

   private:
    std::variant<T, Error> v_;
};

}  // namespace carvetto::post
