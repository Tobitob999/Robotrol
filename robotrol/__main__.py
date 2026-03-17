"""Entry point: python -m robotrol"""

from __future__ import annotations

import tkinter as tk

from robotrol.gui.app import RobotrolApp


def main() -> None:
    """Launch the Robotrol V2.0 GUI application."""
    root = tk.Tk()
    app = RobotrolApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()


if __name__ == "__main__":
    main()
