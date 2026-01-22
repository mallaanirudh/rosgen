# ROSGen

**ROSGen** is a code generation tool for ROS 2 that creates boilerplate ROS 2 packages from a YAML specification. It simplifies package setup, message/service definition, and node scaffolding so you can focus on development instead of boilerplate.

---

## Features

- Generates ROS 2 packages in Python (`ament_python`)  
- Supports custom messages and services  
- Automatically creates nodes with publishers and subscribers  
- Configurable output directory for generated code  

---

## Installation

Clone the repository and install dependencies:

```bash
git clone <repo_url>
cd rosgen
pip install -r requirements.txt
