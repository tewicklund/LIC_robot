from tkinter import Tk, filedialog
import csv
import openpyxl
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def export_grid_as_png(title, grid_chars):
    # Save results grid as png
    output_path = title + '.png'

    # Output grid image dimensions
    rows, cols = 10, 7
    cell_width, cell_height = 1, 1

    # Create the grid
    fig, ax = plt.subplots(figsize=(cols * cell_width, rows * cell_height))
    
    # Draw the grid
    for row in range(rows + 1):
        ax.axhline(y=row, color='black', linewidth=0.5)
    for col in range(cols + 1):
        ax.axvline(x=col, color='black', linewidth=0.5)

    # Add 'X' and 'O' in cells with background for 'X'
    for row in range(rows):
        for col in range(cols):
            if col % 2 == 0:
                grid_number = (10 * col) + 9 - row
            else:
                grid_number = (10 * col) + row

            char = grid_chars[grid_number]
            
            # Add green background for 'X'
            if char == 'X':
                rect = patches.Rectangle(
                    (col, rows - row - 1),  # Bottom-left corner
                    1, 1,  # Width and height
                    color='green',
                    zorder=1  # Ensure it appears below the text
                )
                ax.add_patch(rect)

            # Add text on top
            ax.text(
                col + 0.5, rows - row - 0.5, char,
                ha='center', va='center', fontsize=12, color='black', zorder=2
            )

    # Add title
    fig.suptitle(title, fontsize=16, weight='bold', y=0.95)

    # Add 'START' label in the bottom left cell
    ax.text(0.5, 0.1, "START", ha='center', va='bottom', fontsize=12, weight='bold')

    # Adjust the plot
    expansion_constant = 0.01
    ax.set_xlim(-expansion_constant, cols + expansion_constant)
    ax.set_ylim(-expansion_constant, rows + expansion_constant)
    ax.set_aspect('equal')
    ax.axis('off')  # Hide axes

    # Save the image
    plt.savefig(output_path, bbox_inches='tight', dpi=300)
    plt.close()



def get_timestamp_from_post(stop_ID,stop_type,post_list):
    for post in post_list:
        if post.stop_ID==stop_ID and post.stop_type==stop_type:
            return(post.server_timestamp)
    return 0.0

def csv_to_LIC_post_list(test_ID):
    LIC_post_list=[]
    Tk().withdraw()  # Hide the root window
    file_path = filedialog.askopenfilename(
        title="Select a CSV File",
        filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
    )
    try:
        #open results csv
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            
            # Read each row
            for row in reader:

                # add post to list if it's from the LIC or from the Jetson and has the matching test ID
                if row[1]=='LIC Trigger POST' or row[0]==test_ID:
                    LIC_post_list.append(LIC_post(row[0],row[1],row[2],row[3],row[4],row[5]))


        return LIC_post_list

    except FileNotFoundError:
        print(f"Error: The file '{file_path}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

class LIC_post():
    def __init__(self,test_name,post_type,server_timestamp,client_timestamp,stop_ID,stop_type):
        self.test_name=test_name
        self.post_type=post_type
        self.server_timestamp=round(float(server_timestamp),3)
        self.client_timestamp=round(float(client_timestamp),3)
        self.stop_ID=stop_ID
        self.stop_type=stop_type

    def __str__(self):
        return_string=''
        return_string+='Test Name: '+self.test_name+'\n'
        return_string+='POST Type: '+self.post_type+'\n'
        return_string+='Server Timestamp: '+str(self.server_timestamp)+'\n'
        return_string+='Client Timestamp: '+str(self.client_timestamp)+'\n'
        if self.post_type=='Robot POST':
            return_string+='Stop Number: '+self.stop_ID+'\n'
            return_string+='Arrive/Depart: '+self.stop_type+'\n'
        return return_string
        
        