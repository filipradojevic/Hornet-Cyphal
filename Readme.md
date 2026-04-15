# APP SHELL

This project is intended to be used as a starting point. Make sure to clone
the repository using the ***--recursive*** flag:

```bash
git clone --recursive https://github.com/filipradojevic/Hornet-Cyphal.git
```

Project relies on multiple generators during the build stage, such as mavgen and
ulog-gen, which are included as submodules. Because of these generators, which
are written in Python, it is strongly recommended to create a dedicated Python
virtual environment.

---

## Git Workflow

### Clone the project

```bash
git clone --recursive https://github.com/filipradojevic/Hornet-Cyphal.git
```

### Go to the dir 
```bash
cd Hornet-Cyphal
```


### Create dirs for module that we wanna clone 
```bash
git worktree add ..\GwGnd GwGnd
git worktree add ..\GwSky GwSky
git worktree add ..\ActMaster ActMaster
git worktree add ..\BlackBox BlackBox
git worktree add ..\Ins Ins
git worktree add ..\PwrMan PwrMan
```


### Go back to the all dirs 
```bash
cd ..
```

### Get into cloned dir
```bash
code .\GwGnd
code .\GwSky
code .\ActMaster
code .\BlackBox
code .\Ins
code .\PwrMan
```

### View all branches

```bash
git branch -a
```

### Fetch all remote branches

```bash
git fetch --all
```

### Switch to a branch

```bash
git checkout ActMaster
git checkout BlackBox
git checkout GwGnd
git checkout GwSky
git checkout Ins
git checkout PwrMan
```

### Create a local branch tracking a remote branch

```bash
git checkout -b ActMaster origin/ActMaster
```

### Update the current branch

```bash
git pull
```

### Check the current branch

```bash
git branch
```

---

## Python Virtual Environment

Creating a Python virtual environment named ***.venv*** can be done by running:

- Linux
  ```bash
  python3 -m venv .venv
  ```

- Windows
  ```bash
  python -m venv .venv
  ```

There is no strict naming convention for virtual environments, but the most
commonly used names are ***.venv***, ***venv*** and ***python_venv***.
These names are already added to ***.gitignore***.

### Activate the virtual environment

- Linux
  ```bash
  source .venv/bin/activate
  ```

- Windows
  ```bash
  .venv\Scripts\activate
  ```

### Install dependencies

Make sure that all project requirements are installed in the virtual environment
(such as MAVLink and ULog generator requirements).

**MAVLink Generator**:

- Linux
  ```bash
  python3 -m pip install -r src/middleware/mav/mavlink/pymavlink/requirements.txt
  ```

- Windows
  ```bash
  python -m pip install -r src/middleware/mav/mavlink/pymavlink/requirements.txt
  ```

**ULog Generator**:

- Linux
  ```bash
  python3 -m pip install -r src/middleware/ulog/ulog-gen/requirements.txt
  ```

- Windows
  ```bash
  python -m pip install -r src/middleware/ulog/ulog-gen/requirements.txt
  ```

### Deactivate the virtual environment

```bash
deactivate
```

The virtual environment does not need to be activated again manually — it will
be called by CMake during the build stage.

---

## Code Formatting

`pre-commit` and `clang-format` are used to maintain consistent code formatting.
Before committing your changes, ensure that your code is properly formatted.

* **Install [pre-commit](https://pre-commit.com/):**
  ```bash
  pip install pre-commit
  ```

* **Install Git Hooks:**
  ```bash
  pre-commit install
  ```

Once set up, `pre-commit` will automatically run `clang-format` on your code
before each commit, ensuring consistent formatting.

---

## Tool Configuration

In order to compile the project, the required tools must be configured.
Instructions can be found on QNAP or YouTube:

- [Windows Instructions](https://youtu.be/mzDSuTes94s?si=mTRIQjb0yGjn8cFB)
- [Linux Instructions](https://youtu.be/_yG40rGTXko?si=Ls3UtnsF5oRLxzyv)

### Cortex-Debug

The Cortex-Debug VS Code plugin requires configuration of a `launch.json` file.
More information can be found [here](https://go.microsoft.com/fwlink/?linkid=830387).

Create the file at `.vscode/launch.json` with the following content:

```jsonc
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (ULINK2)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            "cwd": "${workspaceFolder}",
            "device": "LPC1768",
            "configFiles": [
                "${workspaceFolder}/devices/lpc1768/tools/openocd.cfg"
            ],
            "executable": "${workspaceFolder}/build/${workspaceFolderBasename}.elf",
            "svdFile": "${workspaceFolder}/devices/lpc1768/tools/LPC1768.svd",
            "runToEntryPoint": "main",
            "interface": "swd",
            "showDevDebugOutput": "raw",
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 2
            }
        },
        {
            "name": "Debug (JLink)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "jlink",
            "cwd": "${workspaceFolder}",
            "device": "LPC1768",
            "executable": "build/${workspaceFolderBasename}.elf",
            "svdFile": "${workspaceFolder}/devices/lpc1768/tools/LPC1768.svd",
            "runToEntryPoint": "main",
            "interface": "swd",
            "showDevDebugOutput": "raw",
            "liveWatch": {
                "enabled": true,
                "samplesPerSecond": 2
            }
        }
    ]
}
```

> **Note:** The previous version used a hardcoded `program.elf` as the executable name.
> The updated version uses `${workspaceFolderBasename}.elf`, which automatically
> resolves to the project folder name.

---

## Building

To build the project:
1. Choose a preset (depending on the desired optimization level)
2. Build using `F7`

In case of changes in configuration files, make sure to use **Clean Rebuild**.

## Adding a Preset

Use `CMakePresets.json` as a reference.

To add a user preset:
1. Create a `CMakeUserPresets.json` file in the project root.
2. For each preset specify:
   - ***CMAKE_BUILD_TYPE***: CMake build type.