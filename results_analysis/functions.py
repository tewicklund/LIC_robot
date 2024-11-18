from tkinter import Tk, filedialog
import csv
import openpyxl

def get_csv_file():
    Tk().withdraw()  # Hide the root window
    file_path = filedialog.askopenfilename(
        title="Select a CSV File",
        filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
    )
    return file_path