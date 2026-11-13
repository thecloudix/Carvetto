#pragma once

#include <functional>
#include <string>

class QLineEdit;

namespace carvetto::app {

/// "Insert New Document Variable..." on numeric boxes (T-0271). Every numeric edit box of the app
/// calls `attachVariableMenu`; the right-click menu then offers the entry, which asks for a name and
/// a value, creates the variable in the open document (one undo step) and inserts `{Name}` at the
/// cursor, turning the entry into a calculation (`=`) when it is not one already.
///
/// The open window installs the function that creates the variable (empty string = done, else the
/// reason it was refused); with none installed the entry is not offered.
using VariableCreator = std::function<std::string(const std::string& name, const std::string& value)>;
void setVariableCreator(const void* owner, VariableCreator creator);
void clearVariableCreator(const void* owner);

/// Adds the menu entry to `edit` (a no-op when attached already).
void attachVariableMenu(QLineEdit* edit);

/// Creates variable `name` = `value` through the installed creator and inserts `{name}` into `edit`
/// at its cursor, emitting `textEdited`. Returns an empty string, or the reason it failed. The
/// scripted entry of the menu action (the self-test drives it).
std::string insertNewVariable(QLineEdit* edit, const std::string& name, const std::string& value);

}  // namespace carvetto::app
