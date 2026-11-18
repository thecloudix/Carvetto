#include "rotary_post_dialog.hpp"

#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>

namespace carvetto::app {

RotaryPostDialog::RotaryPostDialog(const viewmodel::RotaryPostReport& report, QWidget* parent) : QDialog(parent) {
    setObjectName(QStringLiteral("rotaryPostDialog"));
    setWindowTitle(QCoreApplication::translate("RotaryPostDialog", "Rotary post: refusals and warnings"));
    resize(640, 360);
    auto* layout = new QVBoxLayout(this);
    refused_ = report.refused();
    issues_ = static_cast<int>(report.issues.size());
    auto* heading = new QLabel(
        refused_ ? QCoreApplication::translate("RotaryPostDialog", "The post refused to write this rotary job.")
                 : (report.empty() ? QCoreApplication::translate("RotaryPostDialog", "The post has nothing to say about this rotary job.")
                                   : QCoreApplication::translate("RotaryPostDialog", "The program is written; check these points first.")),
        this);
    heading->setObjectName(QStringLiteral("rotaryPostHeading"));
    heading->setWordWrap(true);
    layout->addWidget(heading);
    list_ = new QListWidget(this);
    list_->setObjectName(QStringLiteral("rotaryPostIssues"));
    list_->setWordWrap(true);
    for (const viewmodel::RotaryPostIssue& i : report.issues) {
        list_->addItem((i.refusal ? QCoreApplication::translate("RotaryPostDialog", "Refused: ") : QCoreApplication::translate("RotaryPostDialog", "Warning: ")) +
                       QString::fromStdString(i.message));
        list_->addItem(QCoreApplication::translate("RotaryPostDialog", "    What to do: ") + QString::fromStdString(i.advice));
    }
    layout->addWidget(list_, 1);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

}  // namespace carvetto::app
