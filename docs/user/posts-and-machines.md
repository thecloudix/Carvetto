# Posts and machines

A post-processor turns toolpaths into the dialect of your controller. Carvetto reads `.pp` post files and ships
posts for Grbl, LinuxCNC, Mach3, Mach4 and Marlin in metric and (for most) inch variants.

- Pick or define machines in **Setup > Tools, Machines and Materials...**; a machine names its post, its
  limits and its tool changing behaviour.
- Output is checked for safety (for example against moves outside the machine limits); see
  [post output safety](../adr/0020-post-output-safety.md).
- To write or change a post, read the [post guide](../../posts/README.md), including its machine-safety checklist.
- Always check a new post with an air cut.
