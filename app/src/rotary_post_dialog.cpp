#include "rotary_post_dialog.hpp"

#include <QDialogButtonBox>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>

namespace carvetto::app {

RotaryPostDialog::RotaryPostDialog(const viewmodel::RotaryPostReport& report, QWidget* parent) : QDialog(parent) {
    setObjectName(QStringLiteral("rotaryPostDialog"));
    setWindowTitle(QStringLiteral("Rotary post: refusals and warnings"));
    resize(640, 360);
    auto* layout = new QVBoxLayout(this);
    refused_ = report.refused();
    issues_ = static_cast<int>(report.issues.size());
    auto* heading = new QLabel(
        refused_ ? QStringLiteral("The post refused to write this rotary job.")
                 : (report.empty() ? QStringLiteral("The post has nothing to say about this rotary job.")
                                   : QStringLiteral("The program is written; check these points first.")),
        this);
    heading->setObjectName(QStringLiteral("rotaryPostHeading"));
    heading->setWordWrap(true);
    layout->addWidget(heading);
    list_ = new QListWidget(this);
    list_->setObjectName(QStringLiteral("rotaryPostIssues"));
    list_->setWordWrap(true);
    for (const viewmodel::RotaryPostIssue& i : report.issues) {
        list_->addItem((i.refusal ? QStringLiteral("Refused: ") : QStringLiteral("Warning: ")) +
                       QString::fromStdString(i.message));
        list_->addItem(QStringLiteral("    What to do: ") + QString::fromStdString(i.advice));
    }
    layout->addWidget(list_, 1);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

}  // namespace carvetto::app
