# Installation

drone_sim needs **Python 3.10 or newer**. The commands below are for
PowerShell on Windows; Linux/macOS differences are noted where they matter.

## 1. Get the code

```powershell
git clone https://github.com/aerlab-deanza/Drone_Simulation.git
cd Drone_Simulation
```

## 2. Create a virtual environment (recommended)

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
```

On Linux or macOS, activate with `source .venv/bin/activate`.

:::{tip}
If PowerShell refuses to run `Activate.ps1`, allow local scripts for your user
once: `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.
:::

## 3. Install the package

```powershell
python -m pip install -e .
```

The `-e` (editable) flag means edits to the source take effect without
reinstalling. Optional extras:

| Command | Adds |
|---|---|
| `python -m pip install -e ".[test]"` | `pytest`, to run the test suite |
| `python -m pip install -e ".[docs]"` | Sphinx and extensions, to build this site |

`requirements.txt` lists the same runtime dependencies (NumPy, SciPy,
Matplotlib, pandas, pyserial) for anyone who prefers not to install the package.

## 4. Check it works

```powershell
python -m pytest
```

All tests and docstring examples should pass. Next: {doc}`quickstart`.
