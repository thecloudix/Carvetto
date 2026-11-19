#!/bin/sh
# Refresh the desktop, MIME and icon caches (no-ops where a tool is missing).
update-mime-database /usr/share/mime >/dev/null 2>&1 || :
update-desktop-database -q /usr/share/applications >/dev/null 2>&1 || :
gtk-update-icon-cache -q -t -f /usr/share/icons/hicolor >/dev/null 2>&1 || :
