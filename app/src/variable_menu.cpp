#include "variable_menu.hpp"

#include <QAction>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenu>
#include <QString>
#include <utility>
#include <vector>

#include "carvetto/viewmodel/calculation.hpp"

namespace carvetto::app {

namespace {

std::vector<std::pair<const void*, VariableCreator>>& creators() {
    static std::vector<std::pair<const void*, VariableCreator>> list;
    return list;
}

const VariableCreator* current() {
    return creators().empty() ? nullptr : &creators().back().second;
}

constexpr const char* kAttachedProperty = "carvettoVariableMenu";

}  // namespace

void setVariableCreator(const void* owner, VariableCreator creator) {
    clearVariableCreator(owner);
    creators().emplace_back(owner, std::move(creator));
}

void clearVariableCreator(const void* owner) {
    std::erase_if(creators(), [owner](const auto& p) { return p.first == owner; });
}

std::string insertNewVariable(QLineEdit* edit, const std::string& name, const std::string& value) {
    const VariableCreator* create = current();
    if (edit == nullptr || create == nullptr || !*create) {
        return "variables are not available";
    }
    const std::string err = (*create)(name, value);
    if (!err.empty()) {
        return err;
    }
    QString text = edit->text();
    int pos = edit->cursorPosition();
    const QString token = QString::fromStdString("{" + name + "}");
    if (!viewmodel::isExpressionText(text.toStdString())) {
        text = QStringLiteral("=") + text;
        ++pos;
    }
    text.insert(pos, token);
    edit->setText(text);
    edit->setCursorPosition(pos + static_cast<int>(token.size()));
    Q_EMIT edit->textEdited(text);
    return {};
}

void attachVariableMenu(QLineEdit* edit) {
    if (edit == nullptr || edit->property(kAttachedProperty).toBool()) {
        return;
    }
    edit->setProperty(kAttachedProperty, true);
    edit->setContextMenuPolicy(Qt::CustomContextMenu);
    QObject::connect(edit, &QLineEdit::customContextMenuRequested, edit, [edit](const QPoint& pos) {
        QMenu* menu = edit->createStandardContextMenu();
        if (current() != nullptr) {
            menu->addSeparator();
            QAction* insert = menu->addAction(QStringLiteral("Insert New Document Variable..."));
            QObject::connect(insert, &QAction::triggered, edit, [edit]() {
                bool ok = false;
                const QString name = QInputDialog::getText(edit, QStringLiteral("New Document Variable"),
                                                           QStringLiteral("Name"), QLineEdit::Normal, QString(), &ok);
                if (!ok) {
                    return;
                }
                const QString value = QInputDialog::getText(edit, QStringLiteral("New Document Variable"),
                                                            QStringLiteral("Value"), QLineEdit::Normal, QString(), &ok);
                if (!ok) {
                    return;
                }
                const std::string err = insertNewVariable(edit, name.toStdString(), value.toStdString());
                if (!err.empty()) {
                    edit->setToolTip(QString::fromStdString(err));
                }
            });
        }
        menu->setAttribute(Qt::WA_DeleteOnClose);
        menu->popup(edit->mapToGlobal(pos));
    });
}

}  // namespace carvetto::app
