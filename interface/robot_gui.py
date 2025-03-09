import tkinter as tk
import threading
from robot_control import move_robot_z  # Import move_robot_z from the other script

# GUI window for Z-axis control
def create_gui():
    root = tk.Tk()
    root.title("Robot Z-Movement Control")

    # Buttons for Z-axis movement
    button_up = tk.Button(root, text="Move +Z", command=lambda: move_robot_z(0.02))
    button_up.pack(pady=10)

    button_down = tk.Button(root, text="Move -Z", command=lambda: move_robot_z(-0.02))
    button_down.pack(pady=10)

    root.mainloop()

# Run GUI in a separate thread
def run_gui():
    gui_thread = threading.Thread(target=create_gui)
    gui_thread.daemon = True  # Ensure it exits when the program ends
    gui_thread.start()
