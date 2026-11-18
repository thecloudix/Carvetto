# Gadgets

Gadgets are small Lua scripts that add menu commands, such as extra shape generators or special toolpaths.
They run in a restricted sandbox: no network, and file access only through the program.

- Use the **Gadgets** menu to run installed gadgets.
- **Gadgets > Install New Gadget...** adds a gadget from a file or folder.
- **Gadgets > Gadget Shortcuts...** assigns keys.
- The [stock gadgets](../gadgets/stock-gadgets.md) ship with the program.
- The scripting API is described in [its design record](../adr/0108-scripting-api-and-gadgets.md).
