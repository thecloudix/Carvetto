#pragma once

#include <QDialog>

#include "carvetto/viewmodel/rotary_post_report.hpp"

class QListWidget;

namespace carvetto::app {

/// Presents the post's rotary refusals and warnings (T-0334): one entry per issue, a refusal marked as
/// such, each followed by what to do about it. Content comes from `viewmodel::rotaryPostReport`.
class RotaryPostDialog : public QDialog {
   public:
    explicit RotaryPostDialog(const viewmodel::RotaryPostReport& report, QWidget* parent = nullptr);

    QListWidget* list() const { return list_; }
    /// Number of issues shown.
    int issueCount() const { return issues_; }
    bool refused() const { return refused_; }

   private:
    QListWidget* list_{nullptr};
    int issues_{0};
    bool refused_{false};
};

}  // namespace carvetto::app
