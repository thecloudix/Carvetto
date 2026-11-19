#!/usr/bin/env python3
"""T-0353: the install tree carries what the Linux packages need.

Runs `cmake --install <build> --prefix <tmp>` and checks the FHS layout, that the desktop entry, MIME type and
icon agree with each other, and that the shipped data sits at ../share/carvetto relative to bin/ (where the app
looks for it). Headless: needs no Qt. Usage: test_linux_install.py <build-dir>
"""
import configparser
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def main() -> int:
    build = os.path.abspath(sys.argv[1])
    prefix = tempfile.mkdtemp(prefix="carvetto-install-")
    errors = []
    try:
        r = subprocess.run(["cmake", "--install", build, "--prefix", prefix], capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout, r.stderr)
            return 1

        def need(rel, directory=False):
            p = os.path.join(prefix, rel)
            if not (os.path.isdir(p) if directory else os.path.isfile(p)):
                errors.append("missing " + rel)
            return p

        has_cli = os.path.isfile(os.path.join(build, "app", "cli", "carvetto-cli"))
        if has_cli:
            need("bin/carvetto-cli")
        need("share/carvetto/posts/grbl_mm.pp")
        need("share/carvetto/gadgets", True)
        need("share/carvetto/tooldb/starter_tools.json")
        need("share/carvetto/licenses/LICENSE")
        need("share/carvetto/licenses/THIRD_PARTY_NOTICES.md")
        icon = need("share/icons/hicolor/scalable/apps/carvetto.svg")
        need("share/icons/hicolor/scalable/mimetypes/application-x-carvetto-project.svg")
        desktop = need("share/applications/carvetto.desktop")
        mime = need("share/mime/packages/carvetto.xml")
        if errors:
            print("\n".join(errors))
            return 1

        # Posts and gadgets resolve at ../share/carvetto from bin/ (shippedPostsDir / shippedGadgetsDir).
        if not os.path.isdir(os.path.normpath(os.path.join(prefix, "bin", "..", "share", "carvetto", "posts"))):
            errors.append("share/carvetto/posts is not reachable from bin/")

        cp = configparser.ConfigParser(interpolation=None)
        cp.optionxform = str
        cp.read(desktop, encoding="utf-8")
        entry = cp["Desktop Entry"]
        for key in ("Type", "Name", "Exec", "Icon", "Categories", "MimeType"):
            if key not in entry:
                errors.append("desktop file lacks " + key)
        if entry.get("Icon") != "carvetto":
            errors.append("desktop Icon must be the installed icon name 'carvetto'")
        if entry.get("Exec", "").split()[0] != "carvetto" or "%F" not in entry.get("Exec", ""):
            errors.append("desktop Exec must run `carvetto %F`")
        if not entry.get("Categories", "").endswith(";") or not entry.get("MimeType", "").endswith(";"):
            errors.append("desktop list values must end with ';'")

        root = ET.parse(mime).getroot()
        ns = "{http://www.freedesktop.org/standards/shared-mime-info}"
        types = {t.get("type"): [g.get("pattern") for g in t.findall(ns + "glob")] for t in root.findall(ns + "mime-type")}
        if types.get("application/x-carvetto-project") != ["*.carv"]:
            errors.append("MIME type application/x-carvetto-project must map exactly *.carv, got %r" % types)
        if "application/x-carvetto-project" not in entry.get("MimeType", "").split(";"):
            errors.append("desktop MimeType does not list the project MIME type")

        svg = ET.parse(icon).getroot()
        if not svg.tag.endswith("svg"):
            errors.append("icon is not an SVG")

        # With a bundled Qt (CARVETTO_BUNDLE_QT) the layout must be self-contained.
        if os.path.isfile(os.path.join(prefix, "bin", "carvetto")):
            conf = os.path.join(prefix, "bin", "qt.conf")
            if os.path.isfile(conf):
                text = open(conf, encoding="utf-8").read()
                if "Plugins = plugins" not in text:
                    errors.append("qt.conf lacks the Plugins path")
                for rel in ("lib/carvetto/plugins/platforms/libqoffscreen.so", "lib/carvetto/plugins/platforms/libqxcb.so"):
                    if not os.path.isfile(os.path.join(prefix, rel)):
                        errors.append("bundled Qt: missing " + rel)
                libs = os.listdir(os.path.join(prefix, "lib", "carvetto", "lib"))
                for want in ("libQt6Core.so", "libQt6Widgets.so", "libQt6Gui.so", "libQt6XcbQpa.so"):
                    if not any(f.startswith(want) for f in libs):
                        errors.append("bundled Qt: missing " + want)
    finally:
        shutil.rmtree(prefix, ignore_errors=True)
    if errors:
        print("\n".join(errors))
        return 1
    print("install layout ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
