# Agent Guidelines

- When adding a new feature, place the implementation in a subfolder of `src/`, creating the subfolder if necessary.
- Avoid simply appending large amounts of code into existing files (often node files); prefer organizing new functionality into appropriately named subdirectories.
- Using ROS dependencies outside of the `nodes/` folder is acceptable but should be minimized when possible.
