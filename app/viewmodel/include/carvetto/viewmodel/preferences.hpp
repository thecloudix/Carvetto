#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/units.hpp"
#include "carvetto/simulate/stock.hpp"

// Qt-free user preferences (T-0348, ADR-0119). One `Preferences` value holds every persisted
// preference with its default; `validate` refuses bad values; `serialize` / `deserialize` map it
// to a flat, versioned key/value list that the app stores in QSettings (the only Qt part).
// `PreferencesModel` is the editable copy the Options dialog works on: `apply` refuses invalid
// values (leaving the model untouched), `restoreDefaults` resets, `load` / `save` go through a
// `PreferencesStore`.

namespace carvetto::viewmodel {

/// Version written under the key "version". Bump it and extend `migrate` when a key changes meaning.
inline constexpr int kPreferencesVersion = 1;

struct Preferences {
    // General
    document::Units defaultUnits{document::Units::Millimetres};  ///< units a new job starts in
    std::string language{"system"};                              ///< "system" or "en" (placeholder, T-0350)
    // Editing
    int undoDepth{200};  ///< undo steps kept per document
    bool autosaveEnabled{true};
    int autosaveMinutes{5};  ///< consumed by the recovery work (T-0357)
    // Grid and snap
    bool showGrid{true};
    double gridSpacingMm{10.0};
    bool snapEnabled{true};
    double snapTolerancePx{10.0};
    // Colours, 0xRRGGBB
    std::uint32_t canvasBackground{0xE2E4E8};
    std::uint32_t materialFill{0xFFFFFF};
    // Display
    bool antialiasing{true};
    bool showRulers{true};
    // Default folders ("" = the system's choice / the last used one)
    std::string openFolder;
    std::string saveFolder;
    std::string exportFolder;
    // Tool database file ("" = the per-user default location)
    std::string toolDatabasePath;
    // Simulation
    simulate::Quality simulationQuality{simulate::Quality::Medium};

    bool operator==(const Preferences&) const = default;
};

/// Pages of the Options dialog, in display order.
enum class PrefCategory { General, Editing, GridSnap, Colours, Display, Folders, Tools, Simulation };

/// One preference as the dialog and the audit see it.
struct PrefField {
    const char* key;  ///< persisted key, "group/name"
    PrefCategory category;
    const char* label;
};

/// Every persisted preference, in dialog order. The self-test checks the dialog edits exactly these.
const std::vector<PrefField>& preferenceFields();
const char* categoryTitle(PrefCategory category);

/// One problem with a value: the key and a message for the user.
struct PrefIssue {
    std::string key;
    std::string message;
};

struct PrefValidation {
    /// Also require folders / the tool database's directory to exist (the dialog does; loading does not).
    bool checkPaths{false};
};

std::vector<PrefIssue> validate(const Preferences& p, const PrefValidation& opts = {});

/// Flat key/value form (string values), version first.
using PrefKeyValues = std::vector<std::pair<std::string, std::string>>;

PrefKeyValues serialize(const Preferences& p);

/// Reads stored values. A missing key keeps its default; an unparsable or invalid one is reported in
/// `issues` and also falls back to its default. A newer "version" than this build knows is reported
/// (known keys are still read). Never fails.
Preferences deserialize(const PrefKeyValues& stored, std::vector<PrefIssue>* issues = nullptr);

/// Upgrades values stored by an older version to the current key set (in place). No step exists yet:
/// version 0 (no "version" key) and 1 share their keys.
void migrate(PrefKeyValues& stored, int fromVersion);

/// Where preferences live. The app implements it over QSettings; tests use `MemoryPreferencesStore`.
class PreferencesStore {
   public:
    virtual ~PreferencesStore() = default;
    virtual PrefKeyValues load() = 0;
    virtual bool save(const PrefKeyValues& values) = 0;
};

class MemoryPreferencesStore : public PreferencesStore {
   public:
    PrefKeyValues load() override { return values; }
    bool save(const PrefKeyValues& v) override {
        values = v;
        return true;
    }
    PrefKeyValues values;
};

class PreferencesModel {
   public:
    const Preferences& current() const { return current_; }
    static Preferences defaults() { return Preferences{}; }

    /// Replaces the preferences when `candidate` is valid (checking paths when `checkPaths`); otherwise
    /// returns the issues and changes nothing. Fires `onChanged` when the value actually changed.
    std::vector<PrefIssue> apply(const Preferences& candidate, bool checkPaths = false);
    /// Back to the defaults (fires `onChanged` if they differ).
    void restoreDefaults();
    /// Loads from the store, migrating older versions; returns what had to be defaulted.
    std::vector<PrefIssue> load(PreferencesStore& store);
    bool save(PreferencesStore& store) const { return store.save(serialize(current_)); }

    std::function<void(const Preferences&)> onChanged;

   private:
    Preferences current_;
};

/// "#rrggbb" for 0xRRGGBB, and back (false when not exactly that form).
std::string colourToString(std::uint32_t rgb);
bool colourFromString(const std::string& text, std::uint32_t& rgb);

}  // namespace carvetto::viewmodel
