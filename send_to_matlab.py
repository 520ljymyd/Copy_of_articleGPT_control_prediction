#!/usr/bin/env python3
"""
send_to_matlab.py

Connect to a shared MATLAB engine and run a .m file in that session.

Usage:
  python send_to_matlab.py <path-to-m-file> [--name ENGINE_NAME]

Before using this script:
  1. Install MATLAB Engine for Python (from your MATLAB installation):
       cd "D:\\MATLAB\\R2023b\\extern\\engines\\python"
       python -m pip install .
  2. In MATLAB (Desktop) run once:
       matlab.engine.shareEngine('vscode')
     You can change the name 'vscode' to any other identifier, but use
     the same name with --name when calling this script.
"""
import sys
import os

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Run a .m file in a shared MATLAB session')
    parser.add_argument('mfile', help='Path to the .m file to run')
    parser.add_argument('--name', default='vscode', help="Shared engine name used in MATLAB (default: 'vscode')")
    args = parser.parse_args()

    mpath = os.path.abspath(args.mfile)
    if not os.path.isfile(mpath):
        print(f"Error: file not found: {mpath}")
        sys.exit(2)

    try:
        import matlab.engine
    except Exception as e:
        print("Error: unable to import matlab.engine. Make sure MATLAB Engine for Python is installed.")
        print("Install by running (in MATLAB installation):")
        print("  cd \"<MATLAB_INSTALL>\\extern\\engines\\python\" && python -m pip install .")
        print("Exception:", e)
        sys.exit(3)

    try:
        print(f"Connecting to shared MATLAB engine '{args.name}'...")
        eng = matlab.engine.connect_matlab(args.name)
    except Exception as e:
        print(f"Error: failed to connect to shared MATLAB engine named '{args.name}'.")
        print("In MATLAB, run: matlab.engine.shareEngine('<name>') to share a session.")
        print("Exception:", e)
        sys.exit(4)

    try:
        # Use MATLAB's run function to execute the file (preserves working directory behavior)
        print(f"Running {mpath} in shared MATLAB session...")
        eng.run(mpath, nargout=0)
        print("Done.")
    except Exception as e:
        print("Error during execution in MATLAB:", e)
        sys.exit(5)
    finally:
        try:
            eng.quit()
        except Exception:
            pass

if __name__ == '__main__':
    main()
